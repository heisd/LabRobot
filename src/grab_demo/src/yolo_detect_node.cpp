// =============================================================================
// yolo_detect_node.cpp
//
// 基于 TensorRT 的 YOLOv8 目标检测节点, 作为 HSV 颜色识别(hsv_range.cpp)的替代方案。
//
// 相机接口与 HSV 节点保持完全一致:
//   订阅彩色  : /camera_arm/color/image_raw   (BGR8)
//   订阅深度  : /camera_arm/depth/image_raw   (16UC1, 单位 mm)
//   订阅内参  : /gemini_info                  (sensor_msgs/CameraInfo)
//   发布 TF   : camera_arm_depth_optical_frame -> target_frame
//
// 这样 grab_service_node 不需要任何改动即可直接复用本节点输出的 target_frame。
//
// 与 HSV 不同点: 默认不弹任何 OpenCV 窗口(headless 安全, 适合 Jetson 无显示器场景),
// 需要可视化时把参数 publish_debug_image 打开, 用 rqt_image_view 订阅 ~/detection_image。
// =============================================================================

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <std_msgs/msg/header.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <NvInfer.h>
#if defined(HAVE_ONNX_PARSER)
#include <NvOnnxParser.h>
#endif
#include <cuda_runtime_api.h>

#include <algorithm>
#include <fstream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// TensorRT 通用辅助
// ---------------------------------------------------------------------------
namespace
{
// CUDA 调用错误检查宏
#define CUDA_CHECK(call)                                                        \
  do {                                                                          \
    cudaError_t err = (call);                                                   \
    if (err != cudaSuccess) {                                                   \
      RCLCPP_ERROR(rclcpp::get_logger("yolo_trt"), "CUDA 错误 %s:%d : %s",      \
                   __FILE__, __LINE__, cudaGetErrorString(err));                \
    }                                                                           \
  } while (0)

// TensorRT 日志器, 只打印 WARNING 及以上
class TrtLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char *msg) noexcept override
  {
    if (severity == Severity::kERROR || severity == Severity::kINTERNAL_ERROR) {
      RCLCPP_ERROR(rclcpp::get_logger("yolo_trt"), "[TensorRT] %s", msg);
    } else if (severity == Severity::kWARNING) {
      RCLCPP_WARN(rclcpp::get_logger("yolo_trt"), "[TensorRT] %s", msg);
    }
  }
};

// 统一的 TensorRT 对象删除器 (TensorRT 8+ 使用 delete 释放接口对象)
struct TrtDeleter
{
  template <typename T>
  void operator()(T *obj) const { delete obj; }
};
template <typename T>
using TrtUniquePtr = std::unique_ptr<T, TrtDeleter>;

// 计算维度的元素总数
int64_t volume(const nvinfer1::Dims &d)
{
  int64_t v = 1;
  for (int i = 0; i < d.nbDims; ++i) v *= d.d[i];
  return v;
}

// COCO 80 类名称, 仅用于日志展示
const std::vector<std::string> kCocoNames = {
  "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
  "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
  "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella",
  "handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite",
  "baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle",
  "wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange",
  "broccoli","carrot","hot dog","pizza","donut","cake","chair","couch","potted plant",
  "bed","dining table","toilet","tv","laptop","mouse","remote","keyboard","cell phone",
  "microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors",
  "teddy bear","hair drier","toothbrush"};

}  // namespace

// ---------------------------------------------------------------------------
// 一个检测框
// ---------------------------------------------------------------------------
struct Detection
{
  cv::Rect box;     // 还原到原始图像坐标的检测框
  float confidence; // 置信度
  int class_id;     // 类别 id
};

// ---------------------------------------------------------------------------
// TensorRT 推理封装: 加载 engine(或从 onnx 构建), 执行 YOLOv8 推理与后处理
// ---------------------------------------------------------------------------
class YoloTensorRT
{
public:
  bool init(const std::string &engine_path, const std::string &onnx_path,
            float conf_thr, float nms_thr)
  {
    conf_thr_ = conf_thr;
    nms_thr_ = nms_thr;

    // 1. 优先加载已有的 .engine 文件; 不存在则尝试用 onnx 构建
    if (!engine_path.empty() && fileExists(engine_path)) {
      if (!loadEngine(engine_path)) return false;
    } else {
#if defined(HAVE_ONNX_PARSER)
      if (onnx_path.empty() || !fileExists(onnx_path)) {
        RCLCPP_ERROR(rclcpp::get_logger("yolo_trt"),
                     "engine(%s) 和 onnx(%s) 都不存在, 无法初始化",
                     engine_path.c_str(), onnx_path.c_str());
        return false;
      }
      RCLCPP_INFO(rclcpp::get_logger("yolo_trt"),
                  "未找到 engine, 正在从 onnx 构建 (首次会比较慢, 之后会缓存到 %s)",
                  engine_path.c_str());
      if (!buildEngineFromOnnx(onnx_path, engine_path)) return false;
#else
      RCLCPP_ERROR(rclcpp::get_logger("yolo_trt"),
                   "未编译 onnx 解析器, 请先用 trtexec 把 onnx 转成 engine 再通过 engine_path 指定");
      return false;
#endif
    }

    context_ = TrtUniquePtr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
    if (!context_) {
      RCLCPP_ERROR(rclcpp::get_logger("yolo_trt"), "创建 IExecutionContext 失败");
      return false;
    }

    return setupBindings();
  }

  // 对一帧 BGR 图像做推理, 返回检测框(已还原到原图坐标)
  std::vector<Detection> infer(const cv::Mat &bgr)
  {
    std::vector<Detection> results;

    // 1. letterbox 预处理(等比缩放 + 灰边填充)
    float scale = 0.0f;
    int pad_x = 0, pad_y = 0;
    cv::Mat input = letterbox(bgr, input_w_, input_h_, scale, pad_x, pad_y);

    // 2. 转成 NCHW float blob, /255, BGR->RGB
    cv::Mat blob = cv::dnn::blobFromImage(input, 1.0 / 255.0,
                                          cv::Size(input_w_, input_h_),
                                          cv::Scalar(), true, false);

    // 3. 拷贝到 GPU, 推理, 拷回结果
    CUDA_CHECK(cudaMemcpyAsync(d_input_, blob.ptr<float>(),
                               input_size_ * sizeof(float),
                               cudaMemcpyHostToDevice, stream_));
    if (!enqueue()) return results;
    CUDA_CHECK(cudaMemcpyAsync(host_output_.data(), d_output_,
                               output_size_ * sizeof(float),
                               cudaMemcpyDeviceToHost, stream_));
    CUDA_CHECK(cudaStreamSynchronize(stream_));

    // 4. 解码 YOLOv8 输出 + NMS
    decode(scale, pad_x, pad_y, bgr.cols, bgr.rows, results);
    return results;
  }

  ~YoloTensorRT()
  {
    if (d_input_) cudaFree(d_input_);
    if (d_output_) cudaFree(d_output_);
    if (stream_) cudaStreamDestroy(stream_);
  }

private:
  static bool fileExists(const std::string &p)
  {
    std::ifstream f(p, std::ios::binary);
    return f.good();
  }

  bool loadEngine(const std::string &engine_path)
  {
    std::ifstream file(engine_path, std::ios::binary);
    if (!file.good()) {
      RCLCPP_ERROR(rclcpp::get_logger("yolo_trt"), "无法打开 engine 文件: %s", engine_path.c_str());
      return false;
    }
    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> data(size);
    file.read(data.data(), size);

    runtime_ = TrtUniquePtr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(logger_));
    engine_ = TrtUniquePtr<nvinfer1::ICudaEngine>(
        runtime_->deserializeCudaEngine(data.data(), size));
    if (!engine_) {
      RCLCPP_ERROR(rclcpp::get_logger("yolo_trt"), "反序列化 engine 失败: %s", engine_path.c_str());
      return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("yolo_trt"), "成功加载 engine: %s", engine_path.c_str());
    return true;
  }

#if defined(HAVE_ONNX_PARSER)
  bool buildEngineFromOnnx(const std::string &onnx_path, const std::string &engine_path)
  {
    auto builder = TrtUniquePtr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(logger_));
#if NV_TENSORRT_MAJOR >= 10
    // TensorRT 10 起 explicit batch 为默认, kEXPLICIT_BATCH 标志已移除
    auto network = TrtUniquePtr<nvinfer1::INetworkDefinition>(
        builder->createNetworkV2(0));
#else
    const auto explicit_batch =
        1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = TrtUniquePtr<nvinfer1::INetworkDefinition>(
        builder->createNetworkV2(explicit_batch));
#endif
    auto parser = TrtUniquePtr<nvonnxparser::IParser>(
        nvonnxparser::createParser(*network, logger_));
    if (!parser->parseFromFile(onnx_path.c_str(),
                               static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
      RCLCPP_ERROR(rclcpp::get_logger("yolo_trt"), "解析 onnx 失败: %s", onnx_path.c_str());
      return false;
    }

    auto config = TrtUniquePtr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
#if (NV_TENSORRT_MAJOR > 8) || (NV_TENSORRT_MAJOR == 8 && NV_TENSORRT_MINOR >= 4)
    config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1ULL << 30);  // 1GB
#else
    config->setMaxWorkspaceSize(1ULL << 30);
#endif
    // Jetson 上开启 FP16 可显著加速
    if (builder->platformHasFastFp16()) {
      config->setFlag(nvinfer1::BuilderFlag::kFP16);
      RCLCPP_INFO(rclcpp::get_logger("yolo_trt"), "启用 FP16 推理");
    }

    auto serialized = TrtUniquePtr<nvinfer1::IHostMemory>(
        builder->buildSerializedNetwork(*network, *config));
    if (!serialized) {
      RCLCPP_ERROR(rclcpp::get_logger("yolo_trt"), "构建序列化 engine 失败");
      return false;
    }

    // 缓存到磁盘, 下次直接加载
    if (!engine_path.empty()) {
      std::ofstream out(engine_path, std::ios::binary);
      out.write(static_cast<const char *>(serialized->data()), serialized->size());
      RCLCPP_INFO(rclcpp::get_logger("yolo_trt"), "engine 已缓存到: %s", engine_path.c_str());
    }

    runtime_ = TrtUniquePtr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(logger_));
    engine_ = TrtUniquePtr<nvinfer1::ICudaEngine>(
        runtime_->deserializeCudaEngine(serialized->data(), serialized->size()));
    return engine_ != nullptr;
  }
#else
  bool buildEngineFromOnnx(const std::string &, const std::string &) { return false; }
#endif

  // 解析输入/输出张量, 分配显存缓冲区
  bool setupBindings()
  {
    nvinfer1::Dims in_dims{}, out_dims{};
#if NV_TENSORRT_MAJOR >= 10
    // TensorRT 10+: 使用按名字的张量 API
    int nb = engine_->getNbIOTensors();
    for (int i = 0; i < nb; ++i) {
      const char *name = engine_->getIOTensorName(i);
      auto dims = engine_->getTensorShape(name);
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
        input_name_ = name; in_dims = dims;
      } else {
        output_name_ = name; out_dims = dims;
      }
    }
#else
    // TensorRT 8/9: 使用 binding 索引 API
    int nb = engine_->getNbBindings();
    for (int i = 0; i < nb; ++i) {
      auto dims = engine_->getBindingDimensions(i);
      if (engine_->bindingIsInput(i)) {
        input_index_ = i; in_dims = dims;
      } else {
        output_index_ = i; out_dims = dims;
      }
    }
#endif

    // 输入维度 NCHW: [1,3,H,W]
    input_h_ = in_dims.d[in_dims.nbDims - 2];
    input_w_ = in_dims.d[in_dims.nbDims - 1];
    input_size_ = volume(in_dims);
    output_size_ = volume(out_dims);

    // YOLOv8 输出一般是 [1, 84, 8400] (channel-major) 或 [1, 8400, 84] (anchor-major)
    int d1 = out_dims.d[1];
    int d2 = out_dims.d[2];
    if (d1 < d2) {            // [1, 84, 8400]
      num_channels_ = d1;
      num_anchors_ = d2;
      anchor_major_ = false;
    } else {                  // [1, 8400, 84]
      num_channels_ = d2;
      num_anchors_ = d1;
      anchor_major_ = true;
    }
    num_classes_ = num_channels_ - 4;

    host_output_.resize(output_size_);
    CUDA_CHECK(cudaMalloc(&d_input_, input_size_ * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_output_, output_size_ * sizeof(float)));
    CUDA_CHECK(cudaStreamCreate(&stream_));

    RCLCPP_INFO(rclcpp::get_logger("yolo_trt"),
                "模型输入 %dx%d, 输出通道=%d 锚点=%d 类别数=%d",
                input_w_, input_h_, num_channels_, num_anchors_, num_classes_);
    return d_input_ && d_output_;
  }

  bool enqueue()
  {
#if NV_TENSORRT_MAJOR >= 10
    context_->setTensorAddress(input_name_.c_str(), d_input_);
    context_->setTensorAddress(output_name_.c_str(), d_output_);
    return context_->enqueueV3(stream_);
#else
    void *bindings[2];
    bindings[input_index_] = d_input_;
    bindings[output_index_] = d_output_;
    return context_->enqueueV2(bindings, stream_, nullptr);
#endif
  }

  // 等比缩放 + 居中灰边填充
  static cv::Mat letterbox(const cv::Mat &src, int dst_w, int dst_h,
                           float &scale, int &pad_x, int &pad_y)
  {
    scale = std::min(static_cast<float>(dst_w) / src.cols,
                     static_cast<float>(dst_h) / src.rows);
    int new_w = static_cast<int>(std::round(src.cols * scale));
    int new_h = static_cast<int>(std::round(src.rows * scale));
    pad_x = (dst_w - new_w) / 2;
    pad_y = (dst_h - new_h) / 2;

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_w, new_h));
    cv::Mat out(dst_h, dst_w, src.type(), cv::Scalar(114, 114, 114));
    resized.copyTo(out(cv::Rect(pad_x, pad_y, new_w, new_h)));
    return out;
  }

  // 读取输出张量中 [通道 c, 锚点 a] 的值
  inline float at(int c, int a) const
  {
    return anchor_major_ ? host_output_[a * num_channels_ + c]
                         : host_output_[c * num_anchors_ + a];
  }

  // 解码 + NMS, 把检测框还原到原图坐标
  void decode(float scale, int pad_x, int pad_y, int img_w, int img_h,
              std::vector<Detection> &results)
  {
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> class_ids;

    for (int a = 0; a < num_anchors_; ++a) {
      // 找该锚点得分最高的类别
      int best_cls = 0;
      float best_score = 0.0f;
      for (int c = 0; c < num_classes_; ++c) {
        float s = at(4 + c, a);
        if (s > best_score) { best_score = s; best_cls = c; }
      }
      if (best_score < conf_thr_) continue;

      // YOLOv8 框格式为 [cx, cy, w, h], 处于 letterbox 后的输入尺度
      float cx = at(0, a);
      float cy = at(1, a);
      float w = at(2, a);
      float h = at(3, a);

      // 还原到原图坐标
      float x0 = (cx - w / 2.0f - pad_x) / scale;
      float y0 = (cy - h / 2.0f - pad_y) / scale;
      float bw = w / scale;
      float bh = h / scale;

      boxes.emplace_back(cv::Rect(cv::Point(static_cast<int>(x0), static_cast<int>(y0)),
                                  cv::Size(static_cast<int>(bw), static_cast<int>(bh))));
      scores.push_back(best_score);
      class_ids.push_back(best_cls);
    }

    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, scores, conf_thr_, nms_thr_, keep);
    for (int idx : keep) {
      Detection det;
      det.box = boxes[idx] & cv::Rect(0, 0, img_w, img_h);  // 裁剪到图像范围
      det.confidence = scores[idx];
      det.class_id = class_ids[idx];
      if (det.box.width > 0 && det.box.height > 0) results.push_back(det);
    }
  }

  // TensorRT 对象
  TrtLogger logger_;
  TrtUniquePtr<nvinfer1::IRuntime> runtime_;
  TrtUniquePtr<nvinfer1::ICudaEngine> engine_;
  TrtUniquePtr<nvinfer1::IExecutionContext> context_;

  // 绑定信息
#if NV_TENSORRT_MAJOR >= 10
  std::string input_name_, output_name_;
#else
  int input_index_ = 0, output_index_ = 1;
#endif

  // 缓冲区
  void *d_input_ = nullptr;
  void *d_output_ = nullptr;
  std::vector<float> host_output_;
  cudaStream_t stream_ = nullptr;

  // 形状参数
  int input_w_ = 640, input_h_ = 640;
  int64_t input_size_ = 0, output_size_ = 0;
  int num_channels_ = 84, num_anchors_ = 8400, num_classes_ = 80;
  bool anchor_major_ = false;

  // 阈值
  float conf_thr_ = 0.25f;
  float nms_thr_ = 0.45f;
};

// ---------------------------------------------------------------------------
// ROS2 节点
// ---------------------------------------------------------------------------
class YoloDetectNode : public rclcpp::Node
{
public:
  YoloDetectNode() : Node("yolo_detect_node")
  {
    // ---- 参数声明(全部可在 launch / 命令行覆盖) ----
    engine_path_ = declare_parameter<std::string>("engine_path", "");
    onnx_path_ = declare_parameter<std::string>("onnx_path", "");
    rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/camera_arm/color/image_raw");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera_arm/depth/image_raw");
    info_topic_ = declare_parameter<std::string>("camera_info_topic", "/gemini_info");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_arm_depth_optical_frame");
    target_frame_ = declare_parameter<std::string>("target_frame", "target_frame");
    conf_thr_ = declare_parameter<double>("conf_threshold", 0.25);
    nms_thr_ = declare_parameter<double>("nms_threshold", 0.45);
    target_class_ = declare_parameter<int>("target_class", -1);  // -1 表示不限类别, 取置信度最高
    z_offset_ = declare_parameter<double>("z_offset", 0.07);     // 与 HSV 保持一致
    show_image_ = declare_parameter<bool>("show_image", false);  // 默认不弹窗
    publish_debug_image_ = declare_parameter<bool>("publish_debug_image", true);

    // ---- 初始化 TensorRT ----
    yolo_ = std::make_shared<YoloTensorRT>();
    if (!yolo_->init(engine_path_, onnx_path_,
                     static_cast<float>(conf_thr_), static_cast<float>(nms_thr_))) {
      RCLCPP_FATAL(get_logger(), "YOLO TensorRT 初始化失败, 节点退出");
      throw std::runtime_error("YOLO TensorRT init failed");
    }

    // ---- 订阅相机内参 ----
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topic_, 1,
        std::bind(&YoloDetectNode::infoCallback, this, std::placeholders::_1));

    // ---- 彩色 + 深度 时间近似同步 (与 HSV 一致) ----
    rgb_sub_.subscribe(this, rgb_topic_);
    depth_sub_.subscribe(this, depth_topic_);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), rgb_sub_, depth_sub_);
    sync_->registerCallback(std::bind(&YoloDetectNode::imageCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));

    tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    if (publish_debug_image_) {
      debug_pub_ = create_publisher<sensor_msgs::msg::Image>("~/detection_image", 1);
    }

    RCLCPP_INFO(get_logger(), "yolo_detect_node 已启动 (target_class=%d, z_offset=%.3f)",
                target_class_, z_offset_);
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

  // 接收相机内参矩阵(与 HSV 节点同样的校验逻辑)
  void infoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg)
  {
    if (camera_info_ready_) return;
    bool k_valid = false;
    for (size_t i = 0; i < msg->k.size(); ++i) {
      if (msg->k[i] != 0) { k_valid = true; break; }
    }
    if (!k_valid) return;

    fx_ = msg->k[0];
    cx_ = msg->k[2];
    fy_ = msg->k[4];
    cy_ = msg->k[5];
    camera_info_ready_ = true;
    RCLCPP_INFO(get_logger(), "已获取相机内参: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
                fx_, fy_, cx_, cy_);
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg)
  {
    if (!camera_info_ready_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "等待相机内参 %s ...",
                           info_topic_.c_str());
      return;
    }

    // ---- 转 OpenCV ----
    cv_bridge::CvImagePtr rgb_ptr, depth_ptr;
    try {
      rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
      depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge 转换失败: %s", e.what());
      return;
    }

    // ---- YOLO 推理 ----
    std::vector<Detection> dets = yolo_->infer(rgb_ptr->image);

    // ---- 选择目标: 指定类别则只在该类中选, 否则在全部检测中选, 取置信度最高 ----
    const Detection *target = nullptr;
    for (const auto &d : dets) {
      if (target_class_ >= 0 && d.class_id != target_class_) continue;
      if (!target || d.confidence > target->confidence) target = &d;
    }

    // ---- 可视化(可选, 默认关闭) ----
    if (show_image_ || (publish_debug_image_ && debug_pub_->get_subscription_count() > 0)) {
      drawAndPublish(rgb_ptr->image, dets, target, rgb_msg->header);
    }

    if (!target) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "未检测到目标物体");
      return;
    }

    // ---- 取检测框中心像素, 读取深度 ----
    int px = target->box.x + target->box.width / 2;
    int py = target->box.y + target->box.height / 2;
    px = std::clamp(px, 0, depth_ptr->image.cols - 1);
    py = std::clamp(py, 0, depth_ptr->image.rows - 1);

    // 中心点附近做中值滤波, 避免单点深度为 0 / 噪声(比 HSV 单点取值更稳)
    double dis = medianDepth(depth_ptr->image, px, py, 5);
    if (dis <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "目标 [%s] 中心深度无效, 跳过本帧",
                           className(target->class_id).c_str());
      return;
    }

    // ---- 针孔模型反投影到相机坐标系 (与 HSV 完全一致) ----
    double x = (px - cx_) / fx_ * dis;
    double y = (py - cy_) / fy_ * dis;
    double z = dis + z_offset_;

    RCLCPP_INFO(get_logger(), "检测到 [%s] conf=%.2f  dis=%.3f  -> (%.3f, %.3f, %.3f)",
                className(target->class_id).c_str(), target->confidence, dis, x, y, z);

    // ---- 发布 TF: camera_frame -> target_frame ----
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->now();
    tf.header.frame_id = camera_frame_;
    tf.child_frame_id = target_frame_;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = z;
    tf.transform.rotation.w = 1.0;  // 仅给位置, 姿态由抓取服务决定
    tf_pub_->sendTransform(tf);
  }

  // 中心 (2r+1)x(2r+1) 窗口内非零深度的中值, 单位米
  double medianDepth(const cv::Mat &depth, int px, int py, int r)
  {
    std::vector<ushort> vals;
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        int x = px + dx, y = py + dy;
        if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows) continue;
        ushort v = depth.at<ushort>(y, x);
        if (v > 0) vals.push_back(v);
      }
    }
    if (vals.empty()) return 0.0;
    std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
    return vals[vals.size() / 2] / 1000.0;  // mm -> m
  }

  std::string className(int id) const
  {
    if (id >= 0 && id < static_cast<int>(kCocoNames.size())) return kCocoNames[id];
    return "id_" + std::to_string(id);
  }

  // 画检测框, 发布到话题, 可选弹窗
  void drawAndPublish(const cv::Mat &image, const std::vector<Detection> &dets,
                      const Detection *target, const std_msgs::msg::Header &header)
  {
    cv::Mat vis = image.clone();
    for (const auto &d : dets) {
      bool is_target = (target && &d == target);
      cv::Scalar color = is_target ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
      cv::rectangle(vis, d.box, color, is_target ? 3 : 2);
      std::string label = className(d.class_id) + cv::format(" %.2f", d.confidence);
      cv::putText(vis, label, cv::Point(d.box.x, std::max(0, d.box.y - 5)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }
    if (target) {  // 目标中心画十字, 与 HSV 风格一致
      int cxp = target->box.x + target->box.width / 2;
      int cyp = target->box.y + target->box.height / 2;
      cv::drawMarker(vis, cv::Point(cxp, cyp), cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 20, 2);
    }

    if (publish_debug_image_ && debug_pub_) {
      auto msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, vis).toImageMsg();
      debug_pub_->publish(*msg);
    }
    if (show_image_) {  // 仅在显式开启时弹窗(默认关闭, headless 安全)
      cv::imshow("YOLO Detection", vis);
      cv::waitKey(1);
    }
  }

  // 成员
  std::shared_ptr<YoloTensorRT> yolo_;
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

  // 参数
  std::string engine_path_, onnx_path_, rgb_topic_, depth_topic_, info_topic_;
  std::string camera_frame_, target_frame_;
  double conf_thr_, nms_thr_, z_offset_;
  int target_class_;
  bool show_image_, publish_debug_image_;

  // 相机内参
  bool camera_info_ready_ = false;
  double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<YoloDetectNode>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("yolo_detect_node"), "节点异常退出: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
