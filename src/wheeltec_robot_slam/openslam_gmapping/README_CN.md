# openslam_gmapping

`OpenSLAM GMapping` 算法库的 ROS2 包装版本。它是粒子滤波 2D SLAM 算法的底层实现,不直接提供 ROS 节点,仅供 `slam_gmapping` 等上层包链接使用。

## 概述

OpenSLAM GMapping 是经典的基于 Rao-Blackwellized Particle Filter 的 2D 激光 SLAM 算法。本包将其源代码组织为 ament/CMake 库,通过 `<build_export_depend>` 与 `<exec_depend>` 暴露给 ROS2 工作空间。

## 目录结构

```
openslam_gmapping/
├── CMakeLists.txt
├── package.xml
├── README                       # 原始英文 README
├── build_tools/
├── include/                     # 头文件
├── particlefilter/              # 粒子滤波
├── gridfastslam/                # GridFastSLAM 主框架
├── grid/                        # 占据栅格结构
├── scanmatcher/                 # 扫描匹配
├── sensor/                      # 传感器抽象(里程计/激光)
├── carmenwrapper/               # CARMEN 数据集兼容层
├── gfs-carmen/                  # CARMEN 工具
├── gui/                         # 原始 GUI(ROS 版未使用)
├── log/、ini/、utils/           # 工具
└── CHANGELOG.rst
```

## 依赖项

- buildtool: `ament_cmake`
- 主要 depend: 标准 C++(无强依赖 ROS2 接口)

## 主要类(库提供)

- `GridSlamProcessor` — SLAM 主流程
- `ScanMatcher`、`ScanMatcherMap` — 扫描匹配器
- `ParticleFilter`、`MotionModel` — 粒子滤波 + 运动模型
- `RangeSensor`、`OdometrySensor` — 传感器抽象
- 占据栅格(`Map`, `Array2D`)、矩阵 / 数学工具

## 编译

```bash
colcon build --packages-select openslam_gmapping
```

编译产物以静态/动态库形式提供给 `slam_gmapping` 链接。

## 使用方式

普通用户无需直接调用本包,只需要在编译 `slam_gmapping` 时把它放入同一个工作空间即可。

```bash
# 同时编译两个包
colcon build --packages-up-to slam_gmapping
source install/setup.bash
ros2 launch slam_gmapping slam_gmapping.launch.py
```

## 注意事项

1. 该库源自 OpenSLAM 项目,许可证为 CC BY-NC-SA 3.0 / GPL,商业使用前请确认许可。
2. 本包不包含 ROS 节点,运行 `ros2 run openslam_gmapping ...` 不会找到任何可执行。
3. 不同编译器/版本可能出现 `-Wreorder`、`-Wsign-compare` 警告,属于历史代码风格,不影响功能。
4. 升级 GCC 后如遇模板相关编译错误,可加 `-fpermissive` 临时绕过,但推荐打补丁修正。
