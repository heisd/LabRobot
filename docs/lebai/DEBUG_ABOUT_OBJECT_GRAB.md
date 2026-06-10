# 原来的机器臂不可以实现我们的颜色识别抓取
## 未修改的源码在这个分支上
```bash
# 切换到原始分支
git checkout WSL 
# 拉取原始分支
git pull lebai WSL
```
## 1.问题一
问题出现在TF树的转换上
我们的TF树看这个文件
[TF](frames_2025-11-12_16.06.38.pdf)
可以看到我们的color_link并没有和我们的base_link建立联系
修改hsv_range.cpp的坐标转换部分
```cpp
obg_msg.header.frame_id="camera_arm_depth_optical_frame";
obg_msg.child_frame_id="target_frame";
```
## 2.问题二
轮廓通过HSV展示可视化程度低
```cpp
// cv_rgb_pptr 
// 初始化:cv_bridge::CvImagePtr cv_rgb_ptr;
// cv_rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
// rhb_msg 是订阅的话题也就是获取的图像 rgb_sub_.subscribe(this, "/camera_arm/color/image_raw");
// 下面是节点初始化的时候对这个进行调用
// approxSync->registerCallback(std::bind(&ImageProcessor::imageCallback, this,std::placeholders::_1, std::placeholders::_2));
//  camera_info_sub=create_subscription<sensor_msgs::msg::CameraInfo>("/gemini_info",1,std::bind(&ImageProcessor::rgb_info_callback,this,std::placeholders::_1));
// 复制从传感器中获取的
cv::Mat display_image=cv_rgb_ptr->image.clone();
// 初始化蓝色十字特征
int cross_size =20; // 画的线长度
int thickness =2;   // 画的线宽度
cv::Scalar blue(255,0,0); //画的线的颜色
// 画水平线
cv::line(display_image,cv::point(newpos.x-cross_size,newpos.y),cv::point(newpos.x+cross_size,newpos.y),blue,thickness);
// 画竖直线
cv::line(display_image,cv::point(newpos.x,newpos.y-cross_size),cv::point(newpos.x,newpos.y+cross_size),blue,thickness);
// 画轮廓线
cv::drawContours(display_image,contours,0,cv::Scalar(0,255,0),2);
// 显示图像
cv::imshow("Detection",display_image);
cv::waitkey(1);
```


