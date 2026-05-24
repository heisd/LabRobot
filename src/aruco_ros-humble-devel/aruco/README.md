# aruco (库)

`aruco` C++ 库的 ROS2 包装包。它来自 University of Cordoba 的 [ArUco 项目](https://www.uco.es/investiga/grupos/ava/portfolio/aruco/),用于检测/估计 ArUco 标记位姿,被 `aruco_ros` 节点链接使用。

## 概述

纯算法库,不提供 ROS 节点。底层实现包含:
- Marker 检测、字典管理(`ARUCO_MIP_36h12`、`ORIGINAL` 等)
- 位姿估计(基于相机内参)
- 高/中/低三档检测模式 (`DM_NORMAL`、`DM_FAST`、`DM_VIDEO_FAST`)

## 目录结构

```
aruco/
├── CMakeLists.txt
├── package.xml
├── CHANGELOG.rst
├── include/aruco/               # 公共头文件
├── src/                         # 算法实现
└── cfg/                         # 内置配置(字典等)
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `OpenCV`(必需)、`Eigen3`

## 提供给上层的 C++ 接口

- `aruco::MarkerDetector` — 主入口
- `aruco::CameraParameters` — 内参
- `aruco::Marker` — 单个检测结果
- `aruco::Dictionary` — 字典
- `aruco::MarkerMapPoseTracker` — Marker Map 跟踪

## 编译

```bash
colcon build --packages-select aruco
```

编译产物以共享库形式安装,可由 `aruco_ros` 等包链接。

## 注意事项

1. 本包是纯库,运行 `ros2 run aruco ...` 不会有可执行文件。
2. OpenCV 版本需与本机一致,否则可能出现 ABI 兼容问题。
3. 若希望命令行调用,可使用 ArUco 上游提供的工具(本包未包含)。
