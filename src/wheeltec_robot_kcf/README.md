# wheeltec_robot_kcf

基于 KCF (Kernelized Correlation Filter) 算法的视觉目标跟踪与机器人跟随系统。

## 📋 目录

- [功能简介](#功能简介)
- [依赖项](#依赖项)
- [编译方法](#编译方法)
- [使用方法](#使用方法)
- [节点信息](#节点信息)
- [参数配置](#参数配置)
- [运行时修改参数](#运行时修改参数)
- [算法说明](#算法说明)
- [常见问题](#常见问题)

## 🎯 功能简介

本包实现了一个完整的视觉目标跟踪与机器人跟随系统，主要功能包括：

- **KCF 视觉跟踪**：基于核相关滤波器的快速目标跟踪算法
- **实时跟踪**：支持 HOG 特征、多尺度跟踪和 LAB 颜色特征
- **深度信息融合**：结合 RGB 图像和深度信息实现精确的距离控制
- **PID 控制**：使用双 PID 控制器分别控制距离和角度
- **交互式目标选择**：通过鼠标在图像窗口中框选目标

## 📦 依赖项

### ROS 2 依赖
- `rclcpp` - ROS 2 C++ 客户端库
- `sensor_msgs` - 传感器消息类型
- `geometry_msgs` - 几何消息类型（Twist）
- `std_msgs` - 标准消息类型
- `cv_bridge` - ROS 图像与 OpenCV 转换
- `image_transport` - 图像传输库

### 系统依赖
- **OpenCV** (3.x 或 4.x) - 计算机视觉库
- **CMake** >= 3.5
- **C++14** 编译器

### 相关包依赖
- `turn_on_wheeltec_robot` - 机器人基础功能包

## 🔨 编译方法

### 1. 克隆或确保包在工作空间中

```bash
cd ~/WHEELTEC_S300Pro_mini
```

### 2. 安装依赖

确保已安装 OpenCV 和所有 ROS 2 依赖：

```bash
sudo apt-get update
sudo apt-get install ros-<distro>-cv-bridge ros-<distro>-image-transport libopencv-dev
```

### 3. 编译工作空间

```bash
cd ~/WHEELTEC_S300Pro_mini
colcon build --packages-select wheeltec_robot_kcf
```

### 4. 配置环境

```bash
source install/setup.bash
```

## 🚀 使用方法

### 方法1：使用 Launch 文件启动（推荐）

```bash
ros2 launch wheeltec_robot_kcf wheeltec_robot_kcf.launch.py
```

该 launch 文件会自动启动：
- 相机节点 (`wheeltec_camera`)
- 机器人控制节点 (`turn_on_wheeltec_robot`)
- KCF 跟踪节点 (`run_tracker_node`)

### 方法2：单独运行节点

首先确保相机和机器人节点已启动，然后运行：

```bash
ros2 run wheeltec_robot_kcf run_tracker_node
```

### 操作流程

1. **启动节点**：使用上述命令启动节点
2. **选择目标**：
   - 如果系统有显示支持（设置了 DISPLAY 环境变量），会自动打开一个图像窗口
   - 在窗口中用鼠标左键拖拽框选要跟踪的目标
   - 释放鼠标后，跟踪自动开始
3. **查看结果**：
   - 跟踪框会显示在图像上（黄色矩形）
   - 目标中心会显示红色圆点
   - 机器人会自动跟随目标移动
4. **停止跟踪**：
   - 在图像窗口中按 `q` 键退出
   - 或使用 `Ctrl+C` 停止节点

## 📡 节点信息

### 节点名称
- `/image_converter`

### 订阅话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB 彩色图像 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图像 (32FC1) |

### 发布话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/KCF_image` | `sensor_msgs/Image` | 可视化后的跟踪结果图像 |
| `/cmd_vel` | `geometry_msgs/Twist` | 机器人速度控制指令 |

## ⚙️ 参数配置

### Launch 文件参数配置

编辑 `launch/wheeltec_robot_kcf.launch.py` 文件，修改参数：

```python
parameters=[{'targetDist_': 0.8}],  # 目标距离（米）
```

### 可用参数

所有参数都可以在运行时动态修改：

| 参数名称 | 类型 | 默认值 | 说明 |
|---------|------|--------|------|
| `targetDist_` | float | 1.0 | 目标跟踪距离（米）|
| `linear_KP_` | float | 3.0 | 线性速度 PID 比例系数 |
| `linear_KI_` | float | 0.0 | 线性速度 PID 积分系数 |
| `linear_KD_` | float | 1.0 | 线性速度 PID 微分系数 |
| `angular_KP_` | float | 0.5 | 角速度 PID 比例系数 |
| `angular_KI_` | float | 0.0 | 角速度 PID 积分系数 |
| `angular_KD_` | float | 2.0 | 角速度 PID 微分系数 |
| `refresh_` | bool | false | 刷新标志 |

## 🔧 运行时修改参数

### 方法1：使用 `ros2 param set`（推荐）

在节点运行时，可以通过命令行动态修改参数：

```bash
# 修改目标距离为 1.0 米
ros2 param set /image_converter targetDist_ 1.0

# 修改为 0.5 米
ros2 param set /image_converter targetDist_ 0.5

# 修改 PID 参数
ros2 param set /image_converter linear_KP_ 3.5
ros2 param set /image_converter angular_KP_ 0.6
```

**注意**：参数修改后会立即生效，因为代码在每次深度回调时都会读取最新的参数值。

### 方法2：查看当前参数

```bash
# 列出所有参数
ros2 param list /image_converter

# 获取特定参数的值
ros2 param get /image_converter targetDist_

# 查看参数描述
ros2 param describe /image_converter targetDist_
```

### 方法3：Launch 时覆盖参数

```bash
ros2 launch wheeltec_robot_kcf wheeltec_robot_kcf.launch.py targetDist_:=1.2
```

更多运行时参数修改方法，请参考 [RUNTIME_PARAM_GUIDE.md](RUNTIME_PARAM_GUIDE.md)。

## 🔬 算法说明

### KCF 跟踪算法

本包实现了基于 **Kernelized Correlation Filter (KCF)** 的目标跟踪算法：

- **算法来源**：基于论文 "High-Speed Tracking with Kernelized Correlation Filters" (TPAMI 2015)
- **核心特点**：
  - 使用循环矩阵和傅里叶变换实现快速相关计算
  - 支持多尺度跟踪
  - 使用 HOG (Histogram of Oriented Gradients) 特征提高鲁棒性
  - 可选 LAB 颜色特征增强

### 特征提取

- **HOG 特征**：提取梯度方向直方图特征（默认启用）
- **LAB 颜色特征**：可选的颜色空间特征
- **多尺度检测**：处理目标大小变化

### 控制算法

使用双 PID 控制器：

1. **线性 PID**：控制机器人到目标之间的距离
   - 目标：维持 `targetDist_` 指定的距离
   - 输入：深度图像计算的实际距离
   - 输出：线性速度 (`linear.x`)

2. **角速度 PID**：控制目标在图像中的位置
   - 目标：将目标保持在图像中心（320 像素）
   - 输入：目标中心 x 坐标
   - 输出：角速度 (`angular.z`)

### 速度限制

为安全起见，代码中限制了最大速度：

- 最大线性速度：±0.35 m/s
- 最大角速度：±0.35 rad/s

## 📁 文件结构

```
wheeltec_robot_kcf/
├── CMakeLists.txt          # CMake 构建配置
├── package.xml             # ROS 2 包配置文件
├── README.md               # 本文件
├── RUNTIME_PARAM_GUIDE.md  # 运行时参数修改指南
├── launch/                 # Launch 文件目录
│   └── wheeltec_robot_kcf.launch.py
├── src/                    # 源代码目录
│   ├── run_tracker.cpp     # 主节点实现
│   ├── kcftracker.cpp      # KCF 跟踪器实现
│   ├── fhog.cpp            # HOG 特征提取
│   └── PID.cpp             # PID 控制器实现
└── include/                # 头文件目录
    └── wheeltec_robot_kcf/
        ├── run_tracker.h
        ├── kcftracker.h
        ├── PID.h
        ├── fhog.h
        └── ...
```

## ❓ 常见问题

### Q1: 无法显示图像窗口

**A**: 如果系统没有显示支持（无 DISPLAY 环境变量），节点会自动运行在无头模式。虽然看不到窗口，但跟踪功能仍然正常工作。

### Q2: 跟踪丢失或漂移

**A**: 可能的原因：
- 目标移动过快
- 光照变化太大
- 目标被遮挡
- 目标颜色与背景太相似

**解决方法**：
- 尝试调整 KCF 跟踪器参数（在代码中）
- 选择对比度高的目标
- 保持适中的跟踪距离

### Q3: 机器人无法到达目标距离

**A**: 检查：
- 深度相机是否正常工作
- 目标距离是否在有效范围内（0.4-10.0 米）
- PID 参数是否需要调整

### Q4: 如何调整跟踪效果

**A**: 可以在代码中修改 KCF 跟踪器参数：

```cpp
// 在 run_tracker.h 中
bool HOG = true;        // 使用 HOG 特征
bool FIXEDWINDOW = false;
bool MULTISCALE = true; // 多尺度跟踪
bool LAB = false;       // LAB 颜色特征
```

### Q5: 如何修改速度限制

**A**: 编辑 `src/run_tracker.cpp` 中的速度限制代码（约第 180-185 行）：

```cpp
if (twist.linear.x > 0.5) {twist.linear.x = 0.35;}  // 修改这里的值
```

## 📝 版本信息

- **版本**: 0.0.0
- **ROS 2 版本**: 支持 Foxy, Galactic, Humble, Iron
- **OpenCV 版本**: 3.x 或 4.x

## 📄 许可证

本包中的 KCF 跟踪算法基于 BSD 3-Clause License。

## 👥 维护者

- Wheeltec

## 🙏 致谢

KCF 跟踪算法实现基于：
- Joao F. Henriques, et al. "High-Speed Tracking with Kernelized Correlation Filters", TPAMI 2015
- 作者：Joao Faro, Christian Bailer, Joao F. Henriques

---

**注意**：使用本包前，请确保相机和机器人控制节点已正确配置并可以正常工作。

