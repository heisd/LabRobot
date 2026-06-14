# slam_gmapping (WheelTec 修改版)

WheelTec 机器人使用的 `slam_gmapping` ROS2 适配版本,基于经典 GMapping 粒子滤波 SLAM,完成 2D 激光雷达建图。

## 概述

该包是 ROS1 `slam_gmapping` 的 ROS2 移植/适配版本,底层依赖 `openslam_gmapping` 算法库。配合 WheelTec 底盘和雷达,即可启动建图。

## 目录结构

```
slam_gmapping/
├── CMakeLists.txt
├── package.xml
├── include/
├── src/
│   └── slam_gmapping.cpp       # 主节点(粒子滤波 SLAM)
└── launch/
    └── slam_gmapping.launch.py # 启动文件
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`nav_msgs`、`sensor_msgs`、`geometry_msgs`、`tf2`、`tf2_ros`、`tf2_geometry_msgs`、`openslam_gmapping`、`message_filters`

## 节点说明

### `slam_gmapping`

订阅:
- `/scan` (sensor_msgs/LaserScan) — 激光雷达扫描数据
- TF: `odom -> base_link`(由底盘提供)

发布:
- `/map` (nav_msgs/OccupancyGrid) — 实时占据栅格地图
- `/map_metadata` (nav_msgs/MapMetaData)
- TF: `map -> odom`(SLAM 输出)

主要参数(默认值可在源码中查阅):
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `base_frame` | `base_link` | 机器人本体坐标系 |
| `map_frame` | `map` | 地图坐标系 |
| `odom_frame` | `odom` | 里程计坐标系 |
| `map_update_interval` | `5.0` | 地图更新间隔(s) |
| `maxUrange` | `16.0` | 雷达最大有效距离 |
| `particles` | `30` | 粒子数 |
| `linearUpdate` / `angularUpdate` | `1.0` / `0.5` | 触发更新的线/角位移阈值 |
| `temporalUpdate` | `-1.0` | 时间更新阈值 |

## 启动文件

### `launch/slam_gmapping.launch.py`

包含 `turn_on_wheeltec_robot` 的底盘启动文件和雷达启动文件,然后启动 `slam_gmapping` 节点,实现"底盘 + 雷达 + GMapping"一体化建图。

参数:
- `use_sim_time` — 默认 `false`,若回放 bag 则需置 `true`。

## 编译与运行

```bash
colcon build --packages-up-to slam_gmapping
source install/setup.bash
ros2 launch slam_gmapping slam_gmapping.launch.py
```

## 使用示例

```bash
# 1. 启动 GMapping 建图(已含底盘 + 雷达)
ros2 launch slam_gmapping slam_gmapping.launch.py

# 2. 启动 RViz 查看地图
ros2 launch wheeltec_rviz2 wheeltec_slam.launch.py

# 3. 用键盘遥控机器人扫一遍环境
ros2 run wheeltec_robot_keyboard wheeltec_keyboard

# 4. 保存地图
ros2 launch wheeltec_nav2 save_map.launch.py
```

## 注意事项

1. GMapping 是粒子滤波算法,对动态环境与大场景效果有限,推荐用于中小室内场景。
2. 粒子数越多越准但 CPU 消耗越高,树莓派类平台建议 30 左右。
3. 该包依赖 `openslam_gmapping`,必须一同编译。
4. 与其他 SLAM(cartographer / slam_toolbox)一次只能选一个运行。

## WSL Humble编译过程中问题
```bash
> colcon build --packages-select orb_slam2_ros

Starting >>> orb_slam2_ros
--- stderr: orb_slam2_ros                           
CMake Error at CMakeLists.txt:28 (find_package):
  By not providing "Findimage_common.cmake" in CMAKE_MODULE_PATH this project
  has asked CMake to find a package configuration file provided by
  "image_common", but CMake did not find one.
```

## 修复安装相应包
```bash
sudo -E apt intsall ros-humble-image-common
```

## 之后
```bash
> colcon build --packages-select orb_slam2_ros

Starting >>> orb_slam2_ros
[Processing: orb_slam2_ros]                         
[Processing: orb_slam2_ros]                                 
[Processing: orb_slam2_ros]                                       
[Processing: orb_slam2_ros]                                       
--- stderr: orb_slam2_ros                                         
CMake Deprecation Warning at /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_target_interfaces.cmake:32 (message):
  Use rosidl_get_typesupport_target() and target_link_libraries() instead of
  rosidl_target_interfaces()
Call Stack (most recent call first):
  CMakeLists.txt:170 (rosidl_target_interfaces)


In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.h:229,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.h:31,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.cpp:27:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp: In member function ‘g2o::SparseBlockMatrix<MatrixType>* g2o::SparseBlockMatrix<MatrixType>::slice(int, int, int, int, bool) const’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:328:9: warning: ISO C++ forbids variable length array ‘rowIdx’ [-Wvla]
  328 |     int rowIdx [m];
      |         ^~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:334:9: warning: ISO C++ forbids variable length array ‘colIdx’ [-Wvla]
  334 |     int colIdx [n];
      |         ^~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp: In member function ‘bool g2o::SparseBlockMatrix<MatrixType>::symmPermutation(g2o::SparseBlockMatrix<MatrixType>*&, const int*, bool) const’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:404:9: warning: ISO C++ forbids variable length array ‘blockSizes’ [-Wvla]
  404 |     int blockSizes[_rowBlockIndices.size()];
      |         ^~~~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:410:9: warning: ISO C++ forbids variable length array ‘pBlockIndices’ [-Wvla]
  410 |     int pBlockIndices[_rowBlockIndices.size()];
      |         ^~~~~~~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.h:229,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/solver.h:32,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/solver.cpp:27:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp: In member function ‘g2o::SparseBlockMatrix<MatrixType>* g2o::SparseBlockMatrix<MatrixType>::slice(int, int, int, int, bool) const’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:328:9: warning: ISO C++ forbids variable length array ‘rowIdx’ [-Wvla]
  328 |     int rowIdx [m];
      |         ^~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:334:9: warning: ISO C++ forbids variable length array ‘colIdx’ [-Wvla]
  334 |     int colIdx [n];
      |         ^~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp: In member function ‘bool g2o::SparseBlockMatrix<MatrixType>::symmPermutation(g2o::SparseBlockMatrix<MatrixType>*&, const int*, bool) const’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:404:9: warning: ISO C++ forbids variable length array ‘blockSizes’ [-Wvla]
  404 |     int blockSizes[_rowBlockIndices.size()];
      |         ^~~~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:410:9: warning: ISO C++ forbids variable length array ‘pBlockIndices’ [-Wvla]
  410 |     int pBlockIndices[_rowBlockIndices.size()];
      |         ^~~~~~~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.h:229,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_optimizer.h:33,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/estimate_propagator.h:31,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/optimizable_graph.cpp:37:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp: In member function ‘g2o::SparseBlockMatrix<MatrixType>* g2o::SparseBlockMatrix<MatrixType>::slice(int, int, int, int, bool) const’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:328:9: warning: ISO C++ forbids variable length array ‘rowIdx’ [-Wvla]
  328 |     int rowIdx [m];
      |         ^~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:334:9: warning: ISO C++ forbids variable length array ‘colIdx’ [-Wvla]
  334 |     int colIdx [n];
      |         ^~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp: In member function ‘bool g2o::SparseBlockMatrix<MatrixType>::symmPermutation(g2o::SparseBlockMatrix<MatrixType>*&, const int*, bool) const’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:404:9: warning: ISO C++ forbids variable length array ‘blockSizes’ [-Wvla]
  404 |     int blockSizes[_rowBlockIndices.size()];
      |         ^~~~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:410:9: warning: ISO C++ forbids variable length array ‘pBlockIndices’ [-Wvla]
  410 |     int pBlockIndices[_rowBlockIndices.size()];
      |         ^~~~~~~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.h:229,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_optimizer.h:33,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/estimate_propagator.h:31,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/estimate_propagator.cpp:27:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp: In member function ‘g2o::SparseBlockMatrix<MatrixType>* g2o::SparseBlockMatrix<MatrixType>::slice(int, int, int, int, bool) const’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:328:9: warning: ISO C++ forbids variable length array ‘rowIdx’ [-Wvla]
  328 |     int rowIdx [m];
      |         ^~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:334:9: warning: ISO C++ forbids variable length array ‘colIdx’ [-Wvla]
  334 |     int colIdx [n];
      |         ^~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp: In member function ‘bool g2o::SparseBlockMatrix<MatrixType>::symmPermutation(g2o::SparseBlockMatrix<MatrixType>*&, const int*, bool) const’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:404:9: warning: ISO C++ forbids variable length array ‘blockSizes’ [-Wvla]
  404 |     int blockSizes[_rowBlockIndices.size()];
      |         ^~~~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:410:9: warning: ISO C++ forbids variable length array ‘pBlockIndices’ [-Wvla]
  410 |     int pBlockIndices[_rowBlockIndices.size()];
      |         ^~~~~~~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.h:229,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_optimizer.h:33,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_optimizer.cpp:27:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp: In member function ‘g2o::SparseBlockMatrix<MatrixType>* g2o::SparseBlockMatrix<MatrixType>::slice(int, int, int, int, bool) const’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:328:9: warning: ISO C++ forbids variable length array ‘rowIdx’ [-Wvla]
  328 |     int rowIdx [m];
      |         ^~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:334:9: warning: ISO C++ forbids variable length array ‘colIdx’ [-Wvla]
  334 |     int colIdx [n];
      |         ^~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp: In member function ‘bool g2o::SparseBlockMatrix<MatrixType>::symmPermutation(g2o::SparseBlockMatrix<MatrixType>*&, const int*, bool) const’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:404:9: warning: ISO C++ forbids variable length array ‘blockSizes’ [-Wvla]
  404 |     int blockSizes[_rowBlockIndices.size()];
      |         ^~~~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:410:9: warning: ISO C++ forbids variable length array ‘pBlockIndices’ [-Wvla]
  410 |     int pBlockIndices[_rowBlockIndices.size()];
      |         ^~~~~~~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.h:229,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/optimization_algorithm.h:37,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.cpp:27:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp: In member function ‘g2o::SparseBlockMatrix<MatrixType>* g2o::SparseBlockMatrix<MatrixType>::slice(int, int, int, int, bool) const’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:328:9: warning: ISO C++ forbids variable length array ‘rowIdx’ [-Wvla]
  328 |     int rowIdx [m];
      |         ^~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:334:9: warning: ISO C++ forbids variable length array ‘colIdx’ [-Wvla]
  334 |     int colIdx [n];
      |         ^~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp: In member function ‘bool g2o::SparseBlockMatrix<MatrixType>::symmPermutation(g2o::SparseBlockMatrix<MatrixType>*&, const int*, bool) const’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:404:9: warning: ISO C++ forbids variable length array ‘blockSizes’ [-Wvla]
  404 |     int blockSizes[_rowBlockIndices.size()];
      |         ^~~~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:410:9: warning: ISO C++ forbids variable length array ‘pBlockIndices’ [-Wvla]
  410 |     int pBlockIndices[_rowBlockIndices.size()];
      |         ^~~~~~~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.h:229,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/optimization_algorithm.h:37,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp:30:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp: In member function ‘g2o::SparseBlockMatrix<MatrixType>* g2o::SparseBlockMatrix<MatrixType>::slice(int, int, int, int, bool) const’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:328:9: warning: ISO C++ forbids variable length array ‘rowIdx’ [-Wvla]
  328 |     int rowIdx [m];
      |         ^~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:334:9: warning: ISO C++ forbids variable length array ‘colIdx’ [-Wvla]
  334 |     int colIdx [n];
      |         ^~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp: In member function ‘bool g2o::SparseBlockMatrix<MatrixType>::symmPermutation(g2o::SparseBlockMatrix<MatrixType>*&, const int*, bool) const’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:404:9: warning: ISO C++ forbids variable length array ‘blockSizes’ [-Wvla]
  404 |     int blockSizes[_rowBlockIndices.size()];
      |         ^~~~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:410:9: warning: ISO C++ forbids variable length array ‘pBlockIndices’ [-Wvla]
  410 |     int pBlockIndices[_rowBlockIndices.size()];
      |         ^~~~~~~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.h:229,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/optimization_algorithm.h:37,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/optimization_algorithm.cpp:27:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp: In member function ‘g2o::SparseBlockMatrix<MatrixType>* g2o::SparseBlockMatrix<MatrixType>::slice(int, int, int, int, bool) const’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:328:9: warning: ISO C++ forbids variable length array ‘rowIdx’ [-Wvla]
  328 |     int rowIdx [m];
      |         ^~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:334:9: warning: ISO C++ forbids variable length array ‘colIdx’ [-Wvla]
  334 |     int colIdx [n];
      |         ^~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp: In member function ‘bool g2o::SparseBlockMatrix<MatrixType>::symmPermutation(g2o::SparseBlockMatrix<MatrixType>*&, const int*, bool) const’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:404:9: warning: ISO C++ forbids variable length array ‘blockSizes’ [-Wvla]
  404 |     int blockSizes[_rowBlockIndices.size()];
      |         ^~~~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:410:9: warning: ISO C++ forbids variable length array ‘pBlockIndices’ [-Wvla]
  410 |     int pBlockIndices[_rowBlockIndices.size()];
      |         ^~~~~~~~~~~~~
In file included from /usr/include/eigen3/Eigen/Core:341,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/jacobian_workspace.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/optimizable_graph.h:41,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/optimizable_graph.cpp:27:
/usr/include/eigen3/Eigen/src/Core/products/TriangularMatrixVector.h: In function ‘static void Eigen::internal::trmv_selector<Mode, 1>::run(const Lhs&, const Rhs&, Dest&, const typename Dest::Scalar&) [with Lhs = Eigen::Transpose<const Eigen::Block<const Eigen::Block<Eigen::Matrix<double, -1, -1>, -1, -1, false>, -1, -1, false> >; Rhs = Eigen::Transpose<const Eigen::CwiseBinaryOp<Eigen::internal::scalar_product_op<double, double>, const Eigen::CwiseNullaryOp<Eigen::internal::scalar_constant_op<double>, const Eigen::Matrix<double, 1, -1> >, const Eigen::Transpose<const Eigen::Block<const Eigen::Block<const Eigen::Block<Eigen::Matrix<double, -1, -1>, -1, -1, false>, -1, 1, true>, -1, 1, false> > > >; Dest = Eigen::Transpose<Eigen::Block<Eigen::Block<Eigen::Matrix<double, -1, -1, 1, -1, -1>, 1, -1, true>, 1, -1, false> >; int Mode = 6]’:
/usr/include/eigen3/Eigen/src/Core/products/TriangularMatrixVector.h:332:12: warning: ‘result’ may be used uninitialized [-Wmaybe-uninitialized]
  327 |     internal::triangular_matrix_vector_product
      |     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  328 |       <Index,Mode,
      |       ~~~~~~~~~~~~
  329 |        LhsScalar, LhsBlasTraits::NeedToConjugate,
      |        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  330 |        RhsScalar, RhsBlasTraits::NeedToConjugate,
      |        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  331 |        RowMajor>
      |        ~~~~~~~~~
  332 |       ::run(actualLhs.rows(),actualLhs.cols(),
      |       ~~~~~^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  333 |             actualLhs.data(),actualLhs.outerStride(),
      |             ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  334 |             actualRhsPtr,1,
      |             ~~~~~~~~~~~~~~~
  335 |             dest.data(),dest.innerStride(),
      |             ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  336 |             actualAlpha);
      |             ~~~~~~~~~~~~
/usr/include/eigen3/Eigen/src/Core/products/TriangularMatrixVector.h:105:24: note: by argument 5 of type ‘const double*’ to ‘static void Eigen::internal::triangular_matrix_vector_product<Index, Mode, LhsScalar, ConjLhs, RhsScalar, ConjRhs, 1, Version>::run(Index, Index, const LhsScalar*, Index, const RhsScalar*, Index, Eigen::internal::triangular_matrix_vector_product<Index, Mode, LhsScalar, ConjLhs, RhsScalar, ConjRhs, 1, Version>::ResScalar*, Index, const ResScalar&) [with Index = long int; int Mode = 6; LhsScalar = double; bool ConjLhs = false; RhsScalar = double; bool ConjRhs = false; int Version = 0]’ declared here
  105 | EIGEN_DONT_INLINE void triangular_matrix_vector_product<Index,Mode,LhsScalar,ConjLhs,RhsScalar,ConjRhs,RowMajor,Version>
      |                        ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In file included from /usr/include/eigen3/Eigen/Core:337,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/jacobian_workspace.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/optimizable_graph.h:41,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/optimizable_graph.cpp:27:
/usr/include/eigen3/Eigen/src/Core/products/SelfadjointMatrixVector.h: In function ‘static void Eigen::internal::selfadjoint_product_impl<Lhs, LhsMode, false, Rhs, 0, true>::run(Dest&, const Lhs&, const Rhs&, const Scalar&) [with Dest = Eigen::Block<Eigen::Matrix<double, -1, 1>, -1, 1, false>; Lhs = Eigen::Block<Eigen::Matrix<double, -1, -1>, -1, -1, false>; int LhsMode = 17; Rhs = Eigen::CwiseBinaryOp<Eigen::internal::scalar_product_op<double, double>, const Eigen::CwiseNullaryOp<Eigen::internal::scalar_constant_op<double>, const Eigen::Matrix<double, -1, 1> >, const Eigen::Block<Eigen::Block<Eigen::Matrix<double, -1, -1>, -1, 1, true>, -1, 1, false> >]’:
/usr/include/eigen3/Eigen/src/Core/products/SelfadjointMatrixVector.h:229:7: warning: ‘result’ may be used uninitialized [-Wmaybe-uninitialized]
  227 |     internal::selfadjoint_matrix_vector_product<Scalar, Index, (internal::traits<ActualLhsTypeCleaned>::Flags&RowMajorBit) ? RowMajor : ColMajor,
      |     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  228 |                                                 int(LhsUpLo), bool(LhsBlasTraits::NeedToConjugate), bool(RhsBlasTraits::NeedToConjugate)>::run
      |                                                 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  229 |       (
      |       ^
  230 |         lhs.rows(),                             // size
      |         ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  231 |         &lhs.coeffRef(0,0),  lhs.outerStride(), // lhs info
      |         ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  232 |         actualRhsPtr,                           // rhs info
      |         ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  233 |         actualDestPtr,                          // result info
      |         ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  234 |         actualAlpha                             // scale factor
      |         ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  235 |       );
      |       ~
/usr/include/eigen3/Eigen/src/Core/products/SelfadjointMatrixVector.h:41:6: note: by argument 4 of type ‘const double*’ to ‘static void Eigen::internal::selfadjoint_matrix_vector_product<Scalar, Index, StorageOrder, UpLo, ConjugateLhs, ConjugateRhs, Version>::run(Index, const Scalar*, Index, const Scalar*, Scalar*, Scalar) [with Scalar = double; Index = long int; int StorageOrder = 0; int UpLo = 1; bool ConjugateLhs = false; bool ConjugateRhs = false; int Version = 0]’ declared here
   41 | void selfadjoint_matrix_vector_product<Scalar,Index,StorageOrder,UpLo,ConjugateLhs,ConjugateRhs,Version>::run(
      |      ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/types/types_six_dof_expmap.cpp: In member function ‘virtual bool g2o::EdgeSE3ProjectXYZ::read(std::istream&)’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/types/types_six_dof_expmap.cpp:82:26: warning: iteration 2 invokes undefined behavior [-Waggressive-loop-optimizations]
   82 |       is >> information()(i,j);
      |             ~~~~~~~~~~~~~^~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/types/types_six_dof_expmap.cpp:81:20: note: within this loop
   81 |     for (int j=i; j<2; j++) {
      |                   ~^~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/types/types_seven_dof_expmap.cpp: In member function ‘virtual bool g2o::EdgeSim3ProjectXYZ::read(std::istream&)’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/types/types_seven_dof_expmap.cpp:139:22: warning: iteration 2 invokes undefined behavior [-Waggressive-loop-optimizations]
  139 |   is >> information()(i,j);
      |         ~~~~~~~~~~~~~^~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/types/types_seven_dof_expmap.cpp:138:22: note: within this loop
  138 |       for (int j=i; j<2; j++) {
      |                     ~^~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/BoostArchiver.h:13,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrameDatabase.h:31,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/ORBmatcher.h:29,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/ORBmatcher.cc:21:
/usr/include/boost/serialization/list.hpp: In function ‘void boost::serialization::load(Archive&, std::__cxx11::list<_ValT, _Allocator>&, unsigned int)’:
/usr/include/boost/serialization/list.hpp:53:33: error: ‘library_version_type’ in namespace ‘boost::serialization’ does not name a type; did you mean ‘item_version_type’?
   53 |     const boost::serialization::library_version_type library_version(
      |                                 ^~~~~~~~~~~~~~~~~~~~
      |                                 item_version_type
/usr/include/boost/serialization/list.hpp:60:30: error: ‘library_version_type’ is not a member of ‘boost::serialization’; did you mean ‘item_version_type’?
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                              ^~~~~~~~~~~~~~~~~~~~
      |                              item_version_type
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/BoostArchiver.h:13,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrameDatabase.h:31,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/MapPoint.cc:21:
/usr/include/boost/serialization/list.hpp: In function ‘void boost::serialization::load(Archive&, std::__cxx11::list<_ValT, _Allocator>&, unsigned int)’:
/usr/include/boost/serialization/list.hpp:53:33: error: ‘library_version_type’ in namespace ‘boost::serialization’ does not name a type; did you mean ‘item_version_type’?
   53 |     const boost::serialization::library_version_type library_version(
      |                                 ^~~~~~~~~~~~~~~~~~~~
      |                                 item_version_type
/usr/include/boost/serialization/list.hpp:60:30: error: ‘library_version_type’ is not a member of ‘boost::serialization’; did you mean ‘item_version_type’?
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                              ^~~~~~~~~~~~~~~~~~~~
      |                              item_version_type
/usr/include/boost/serialization/list.hpp:60:56: error: ‘library_version’ was not declared in this scope
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                                                        ^~~~~~~~~~~~~~~
/usr/include/boost/serialization/list.hpp:60:56: error: ‘library_version’ was not declared in this scope
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                                                        ^~~~~~~~~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/BoostArchiver.h:13,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrameDatabase.h:31,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Frame.h:26,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Frame.cc:21:
/usr/include/boost/serialization/list.hpp: In function ‘void boost::serialization::load(Archive&, std::__cxx11::list<_ValT, _Allocator>&, unsigned int)’:
/usr/include/boost/serialization/list.hpp:53:33: error: ‘library_version_type’ in namespace ‘boost::serialization’ does not name a type; did you mean ‘item_version_type’?
   53 |     const boost::serialization::library_version_type library_version(
      |                                 ^~~~~~~~~~~~~~~~~~~~
      |                                 item_version_type
/usr/include/boost/serialization/list.hpp:60:30: error: ‘library_version_type’ is not a member of ‘boost::serialization’; did you mean ‘item_version_type’?
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                              ^~~~~~~~~~~~~~~~~~~~
      |                              item_version_type
/usr/include/boost/serialization/list.hpp:60:56: error: ‘library_version’ was not declared in this scope
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                                                        ^~~~~~~~~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/BoostArchiver.h:13,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Map.h:26,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:26,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/KeyFrame.cc:21:
/usr/include/boost/serialization/list.hpp: In function ‘void boost::serialization::load(Archive&, std::__cxx11::list<_ValT, _Allocator>&, unsigned int)’:
/usr/include/boost/serialization/list.hpp:53:33: error: ‘library_version_type’ in namespace ‘boost::serialization’ does not name a type; did you mean ‘item_version_type’?
   53 |     const boost::serialization::library_version_type library_version(
      |                                 ^~~~~~~~~~~~~~~~~~~~
      |                                 item_version_type
/usr/include/boost/serialization/list.hpp:60:30: error: ‘library_version_type’ is not a member of ‘boost::serialization’; did you mean ‘item_version_type’?
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                              ^~~~~~~~~~~~~~~~~~~~
      |                              item_version_type
/usr/include/boost/serialization/list.hpp:60:56: error: ‘library_version’ was not declared in this scope
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                                                        ^~~~~~~~~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/BoostArchiver.h:13,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Map.h:26,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:26,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrameDatabase.h:28,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/KeyFrameDatabase.cc:21:
/usr/include/boost/serialization/list.hpp: In function ‘void boost::serialization::load(Archive&, std::__cxx11::list<_ValT, _Allocator>&, unsigned int)’:
/usr/include/boost/serialization/list.hpp:53:33: error: ‘library_version_type’ in namespace ‘boost::serialization’ does not name a type; did you mean ‘item_version_type’?
   53 |     const boost::serialization::library_version_type library_version(
      |                                 ^~~~~~~~~~~~~~~~~~~~
      |                                 item_version_type
/usr/include/boost/serialization/list.hpp:60:30: error: ‘library_version_type’ is not a member of ‘boost::serialization’; did you mean ‘item_version_type’?
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                              ^~~~~~~~~~~~~~~~~~~~
      |                              item_version_type
/usr/include/boost/serialization/list.hpp:60:56: error: ‘library_version’ was not declared in this scope
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                                                        ^~~~~~~~~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/BoostArchiver.h:13,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Map.h:26,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:26,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Sim3Solver.h:29,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Sim3Solver.cc:22:
/usr/include/boost/serialization/list.hpp: In function ‘void boost::serialization::load(Archive&, std::__cxx11::list<_ValT, _Allocator>&, unsigned int)’:
/usr/include/boost/serialization/list.hpp:53:33: error: ‘library_version_type’ in namespace ‘boost::serialization’ does not name a type; did you mean ‘item_version_type’?
   53 |     const boost::serialization::library_version_type library_version(
      |                                 ^~~~~~~~~~~~~~~~~~~~
      |                                 item_version_type
/usr/include/boost/serialization/list.hpp:60:30: error: ‘library_version_type’ is not a member of ‘boost::serialization’; did you mean ‘item_version_type’?
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                              ^~~~~~~~~~~~~~~~~~~~
      |                              item_version_type
/usr/include/boost/serialization/list.hpp:60:56: error: ‘library_version’ was not declared in this scope
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                                                        ^~~~~~~~~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/BoostArchiver.h:13,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrameDatabase.h:31,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/PnPsolver.h:56,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/PnPsolver.cc:53:
/usr/include/boost/serialization/list.hpp: In function ‘void boost::serialization::load(Archive&, std::__cxx11::list<_ValT, _Allocator>&, unsigned int)’:
/usr/include/boost/serialization/list.hpp:53:33: error: ‘library_version_type’ in namespace ‘boost::serialization’ does not name a type; did you mean ‘item_version_type’?
   53 |     const boost::serialization::library_version_type library_version(
      |                                 ^~~~~~~~~~~~~~~~~~~~
      |                                 item_version_type
/usr/include/boost/serialization/list.hpp:60:30: error: ‘library_version_type’ is not a member of ‘boost::serialization’; did you mean ‘item_version_type’?
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                              ^~~~~~~~~~~~~~~~~~~~
      |                              item_version_type
/usr/include/boost/serialization/list.hpp:60:56: error: ‘library_version’ was not declared in this scope
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                                                        ^~~~~~~~~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/BoostArchiver.h:13,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrameDatabase.h:31,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Map.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Map.cc:21:
/usr/include/boost/serialization/list.hpp: In function ‘void boost::serialization::load(Archive&, std::__cxx11::list<_ValT, _Allocator>&, unsigned int)’:
/usr/include/boost/serialization/list.hpp:53:33: error: ‘library_version_type’ in namespace ‘boost::serialization’ does not name a type; did you mean ‘item_version_type’?
   53 |     const boost::serialization::library_version_type library_version(
      |                                 ^~~~~~~~~~~~~~~~~~~~
      |                                 item_version_type
/usr/include/boost/serialization/list.hpp:60:30: error: ‘library_version_type’ is not a member of ‘boost::serialization’; did you mean ‘item_version_type’?
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                              ^~~~~~~~~~~~~~~~~~~~
      |                              item_version_type
/usr/include/boost/serialization/list.hpp:60:56: error: ‘library_version’ was not declared in this scope
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                                                        ^~~~~~~~~~~~~~~
gmake[2]: *** [CMakeFiles/orb_slam2_ros_core.dir/build.make:146: CMakeFiles/orb_slam2_ros_core.dir/orb_slam2/src/ORBmatcher.cc.o] Error 1
gmake[2]: *** Waiting for unfinished jobs....
gmake[2]: *** [CMakeFiles/orb_slam2_ros_core.dir/build.make:188: CMakeFiles/orb_slam2_ros_core.dir/orb_slam2/src/MapPoint.cc.o] Error 1
gmake[2]: *** [CMakeFiles/orb_slam2_ros_core.dir/build.make:272: CMakeFiles/orb_slam2_ros_core.dir/orb_slam2/src/KeyFrameDatabase.cc.o] Error 1
gmake[2]: *** [CMakeFiles/orb_slam2_ros_core.dir/build.make:286: CMakeFiles/orb_slam2_ros_core.dir/orb_slam2/src/Sim3Solver.cc.o] Error 1
gmake[2]: *** [CMakeFiles/orb_slam2_ros_core.dir/build.make:244: CMakeFiles/orb_slam2_ros_core.dir/orb_slam2/src/PnPsolver.cc.o] Error 1
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/BoostArchiver.h:13,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Map.h:26,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:26,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/LocalMapping.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/LocalMapping.cc:21:
/usr/include/boost/serialization/list.hpp: In function ‘void boost::serialization::load(Archive&, std::__cxx11::list<_ValT, _Allocator>&, unsigned int)’:
/usr/include/boost/serialization/list.hpp:53:33: error: ‘library_version_type’ in namespace ‘boost::serialization’ does not name a type; did you mean ‘item_version_type’?
   53 |     const boost::serialization::library_version_type library_version(
      |                                 ^~~~~~~~~~~~~~~~~~~~
      |                                 item_version_type
/usr/include/boost/serialization/list.hpp:60:30: error: ‘library_version_type’ is not a member of ‘boost::serialization’; did you mean ‘item_version_type’?
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                              ^~~~~~~~~~~~~~~~~~~~
      |                              item_version_type
gmake[2]: *** [CMakeFiles/orb_slam2_ros_core.dir/build.make:216: CMakeFiles/orb_slam2_ros_core.dir/orb_slam2/src/Map.cc.o] Error 1
/usr/include/boost/serialization/list.hpp:60:56: error: ‘library_version’ was not declared in this scope
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                                                        ^~~~~~~~~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/BoostArchiver.h:13,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrameDatabase.h:31,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/FrameDrawer.h:25,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Tracking.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/System.h:32,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/System.cc:23:
/usr/include/boost/serialization/list.hpp: In function ‘void boost::serialization::load(Archive&, std::__cxx11::list<_ValT, _Allocator>&, unsigned int)’:
/usr/include/boost/serialization/list.hpp:53:33: error: ‘library_version_type’ in namespace ‘boost::serialization’ does not name a type; did you mean ‘item_version_type’?
   53 |     const boost::serialization::library_version_type library_version(
      |                                 ^~~~~~~~~~~~~~~~~~~~
      |                                 item_version_type
/usr/include/boost/serialization/list.hpp:60:30: error: ‘library_version_type’ is not a member of ‘boost::serialization’; did you mean ‘item_version_type’?
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                              ^~~~~~~~~~~~~~~~~~~~
      |                              item_version_type
/usr/include/boost/serialization/list.hpp:60:56: error: ‘library_version’ was not declared in this scope
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                                                        ^~~~~~~~~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/BoostArchiver.h:13,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Map.h:26,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:26,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/LoopClosing.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/LoopClosing.cc:21:
/usr/include/boost/serialization/list.hpp: In function ‘void boost::serialization::load(Archive&, std::__cxx11::list<_ValT, _Allocator>&, unsigned int)’:
/usr/include/boost/serialization/list.hpp:53:33: error: ‘library_version_type’ in namespace ‘boost::serialization’ does not name a type; did you mean ‘item_version_type’?
   53 |     const boost::serialization::library_version_type library_version(
      |                                 ^~~~~~~~~~~~~~~~~~~~
      |                                 item_version_type
/usr/include/boost/serialization/list.hpp:60:30: error: ‘library_version_type’ is not a member of ‘boost::serialization’; did you mean ‘item_version_type’?
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                              ^~~~~~~~~~~~~~~~~~~~
      |                              item_version_type
/usr/include/boost/serialization/list.hpp:60:56: error: ‘library_version’ was not declared in this scope
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                                                        ^~~~~~~~~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/BoostArchiver.h:13,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrameDatabase.h:31,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/FrameDrawer.h:25,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Tracking.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:22:
/usr/include/boost/serialization/list.hpp: In function ‘void boost::serialization::load(Archive&, std::__cxx11::list<_ValT, _Allocator>&, unsigned int)’:
/usr/include/boost/serialization/list.hpp:53:33: error: ‘library_version_type’ in namespace ‘boost::serialization’ does not name a type; did you mean ‘item_version_type’?
   53 |     const boost::serialization::library_version_type library_version(
      |                                 ^~~~~~~~~~~~~~~~~~~~
      |                                 item_version_type
/usr/include/boost/serialization/list.hpp:60:30: error: ‘library_version_type’ is not a member of ‘boost::serialization’; did you mean ‘item_version_type’?
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                              ^~~~~~~~~~~~~~~~~~~~
      |                              item_version_type
/usr/include/boost/serialization/list.hpp:60:56: error: ‘library_version’ was not declared in this scope
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                                                        ^~~~~~~~~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/BoostArchiver.h:13,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrameDatabase.h:31,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Map.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Optimizer.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Optimizer.cc:21:
/usr/include/boost/serialization/list.hpp: In function ‘void boost::serialization::load(Archive&, std::__cxx11::list<_ValT, _Allocator>&, unsigned int)’:
/usr/include/boost/serialization/list.hpp:53:33: error: ‘library_version_type’ in namespace ‘boost::serialization’ does not name a type; did you mean ‘item_version_type’?
   53 |     const boost::serialization::library_version_type library_version(
      |                                 ^~~~~~~~~~~~~~~~~~~~
      |                                 item_version_type
/usr/include/boost/serialization/list.hpp:60:30: error: ‘library_version_type’ is not a member of ‘boost::serialization’; did you mean ‘item_version_type’?
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                              ^~~~~~~~~~~~~~~~~~~~
      |                              item_version_type
/usr/include/boost/serialization/list.hpp:60:56: error: ‘library_version’ was not declared in this scope
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                                                        ^~~~~~~~~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/BoostArchiver.h:13,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrameDatabase.h:31,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Map.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Tracking.h:31,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/FrameDrawer.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/FrameDrawer.cc:21:
/usr/include/boost/serialization/list.hpp: In function ‘void boost::serialization::load(Archive&, std::__cxx11::list<_ValT, _Allocator>&, unsigned int)’:
/usr/include/boost/serialization/list.hpp:53:33: error: ‘library_version_type’ in namespace ‘boost::serialization’ does not name a type; did you mean ‘item_version_type’?
   53 |     const boost::serialization::library_version_type library_version(
      |                                 ^~~~~~~~~~~~~~~~~~~~
      |                                 item_version_type
/usr/include/boost/serialization/list.hpp:60:30: error: ‘library_version_type’ is not a member of ‘boost::serialization’; did you mean ‘item_version_type’?
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                              ^~~~~~~~~~~~~~~~~~~~
      |                              item_version_type
/usr/include/boost/serialization/list.hpp:60:56: error: ‘library_version’ was not declared in this scope
   60 |     if(boost::serialization::library_version_type(3) < library_version){
      |                                                        ^~~~~~~~~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/System.cc:23:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/System.h: In constructor ‘ORB_SLAM2::System::System(std::string, ORB_SLAM2::System::eSensor, ORB_SLAM2::ORBParameters&, const string&, bool)’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/System.h:208:10: warning: ‘ORB_SLAM2::System::mbDeactivateLocalizationMode’ will be initialized after [-Wreorder]
  208 |     bool mbDeactivateLocalizationMode;
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/System.h:168:17: warning:   ‘std::string ORB_SLAM2::System::map_file’ [-Wreorder]
  168 |     std::string map_file;
      |                 ^~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/System.cc:31:1: warning:   when initialized here [-Wreorder]
   31 | System::System(const string strVocFile, const eSensor sensor, ORBParameters& parameters,
      | ^~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/System.cc:23:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/System.h:168:17: warning: ‘ORB_SLAM2::System::map_file’ will be initialized after [-Wreorder]
  168 |     std::string map_file;
      |                 ^~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/System.h:166:10: warning:   ‘bool ORB_SLAM2::System::load_map’ [-Wreorder]
  166 |     bool load_map;
      |          ^~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/System.cc:31:1: warning:   when initialized here [-Wreorder]
   31 | System::System(const string strVocFile, const eSensor sensor, ORBParameters& parameters,
      | ^~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/LoopClosing.cc:21:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/LoopClosing.h: In constructor ‘ORB_SLAM2::LoopClosing::LoopClosing(ORB_SLAM2::Map*, ORB_SLAM2::KeyFrameDatabase*, ORB_SLAM2::ORBVocabulary*, bool, std::shared_ptr<ORB_SLAM2::PointCloudMapping>)’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/LoopClosing.h:107:10: warning: ‘ORB_SLAM2::LoopClosing::mpMap’ will be initialized after [-Wreorder]
  107 |     Map* mpMap;
      |          ^~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/LoopClosing.h:69:36: warning:   ‘std::shared_ptr<ORB_SLAM2::PointCloudMapping> ORB_SLAM2::LoopClosing::mpPointCloudMapping’ [-Wreorder]
   69 |     shared_ptr<PointCloudMapping>  mpPointCloudMapping;
      |                                    ^~~~~~~~~~~~~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/LoopClosing.cc:40:1: warning:   when initialized here [-Wreorder]
   40 | LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale,shared_ptr<PointCloudMapping> pPointCloud):
      | ^~~~~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:22:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Tracking.h: In constructor ‘ORB_SLAM2::Tracking::Tracking(ORB_SLAM2::System*, ORB_SLAM2::ORBVocabulary*, ORB_SLAM2::FrameDrawer*, ORB_SLAM2::Map*, std::shared_ptr<ORB_SLAM2::PointCloudMapping>, ORB_SLAM2::KeyFrameDatabase*, int, ORB_SLAM2::ORBParameters&)’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Tracking.h:248:35: warning: ‘ORB_SLAM2::Tracking::mpPointCloudMapping’ will be initialized after [-Wreorder]
  248 |     shared_ptr<PointCloudMapping> mpPointCloudMapping;
      |                                   ^~~~~~~~~~~~~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Tracking.h:186:23: warning:   ‘ORB_SLAM2::KeyFrameDatabase* ORB_SLAM2::Tracking::mpKeyFrameDB’ [-Wreorder]
  186 |     KeyFrameDatabase* mpKeyFrameDB;
      |                       ^~~~~~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:46:1: warning:   when initialized here [-Wreorder]
   46 | Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer,
      | ^~~~~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:22:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Tracking.h:229:18: warning: ‘ORB_SLAM2::Tracking::mnLastRelocFrameId’ will be initialized after [-Wreorder]
  229 |     unsigned int mnLastRelocFrameId;
      |                  ^~~~~~~~~~~~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Tracking.h:174:9: warning:   ‘int ORB_SLAM2::Tracking::mnMinimumKeyFrames’ [-Wreorder]
  174 |     int mnMinimumKeyFrames;
      |         ^~~~~~~~~~~~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:46:1: warning:   when initialized here [-Wreorder]
   46 | Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer,
      | ^~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc: In member function ‘cv::Mat ORB_SLAM2::Tracking::GrabImageStereo(const cv::Mat&, const cv::Mat&, const double&)’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:181:137: warning: implicitly-declared ‘ORB_SLAM2::Frame& ORB_SLAM2::Frame::operator=(const ORB_SLAM2::Frame&)’ is deprecated [-Wdeprecated-copy]
  181 |     mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
      |                                                                                                                                         ^
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:29,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/FrameDrawer.h:25,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Tracking.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:22:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Frame.h:49:5: note: because ‘ORB_SLAM2::Frame’ has user-provided ‘ORB_SLAM2::Frame::Frame(const ORB_SLAM2::Frame&)’
   49 |     Frame(const Frame &frame);
      |     ^~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc: In member function ‘cv::Mat ORB_SLAM2::Tracking::GrabImageRGBD(const cv::Mat&, const cv::Mat&, const double&)’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:213:114: warning: implicitly-declared ‘ORB_SLAM2::Frame& ORB_SLAM2::Frame::operator=(const ORB_SLAM2::Frame&)’ is deprecated [-Wdeprecated-copy]
  213 |     mCurrentFrame = Frame(mImGray,mImDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
      |                                                                                                                  ^
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:29,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/FrameDrawer.h:25,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Tracking.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:22:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Frame.h:49:5: note: because ‘ORB_SLAM2::Frame’ has user-provided ‘ORB_SLAM2::Frame::Frame(const ORB_SLAM2::Frame&)’
   49 |     Frame(const Frame &frame);
      |     ^~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc: In member function ‘cv::Mat ORB_SLAM2::Tracking::GrabImageMonocular(const cv::Mat&, const double&)’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:241:108: warning: implicitly-declared ‘ORB_SLAM2::Frame& ORB_SLAM2::Frame::operator=(const ORB_SLAM2::Frame&)’ is deprecated [-Wdeprecated-copy]
  241 |         mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
      |                                                                                                            ^
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:29,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/FrameDrawer.h:25,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Tracking.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:22:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Frame.h:49:5: note: because ‘ORB_SLAM2::Frame’ has user-provided ‘ORB_SLAM2::Frame::Frame(const ORB_SLAM2::Frame&)’
   49 |     Frame(const Frame &frame);
      |     ^~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:243:109: warning: implicitly-declared ‘ORB_SLAM2::Frame& ORB_SLAM2::Frame::operator=(const ORB_SLAM2::Frame&)’ is deprecated [-Wdeprecated-copy]
  243 |         mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
      |                                                                                                             ^
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:29,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/FrameDrawer.h:25,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Tracking.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:22:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Frame.h:49:5: note: because ‘ORB_SLAM2::Frame’ has user-provided ‘ORB_SLAM2::Frame::Frame(const ORB_SLAM2::Frame&)’
   49 |     Frame(const Frame &frame);
      |     ^~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc: In member function ‘void ORB_SLAM2::Tracking::Track()’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:457:39: warning: comparison of integer expressions of different signedness: ‘long unsigned int’ and ‘int’ [-Wsign-compare]
  457 |             if(mpMap->KeyFramesInMap()<=mnMinimumKeyFrames)
      |                ~~~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~~~~~~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:468:41: warning: implicitly-declared ‘ORB_SLAM2::Frame& ORB_SLAM2::Frame::operator=(const ORB_SLAM2::Frame&)’ is deprecated [-Wdeprecated-copy]
  468 |         mLastFrame = Frame(mCurrentFrame);
      |                                         ^
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:29,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/FrameDrawer.h:25,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Tracking.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:22:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Frame.h:49:5: note: because ‘ORB_SLAM2::Frame’ has user-provided ‘ORB_SLAM2::Frame::Frame(const ORB_SLAM2::Frame&)’
   49 |     Frame(const Frame &frame);
      |     ^~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc: In member function ‘void ORB_SLAM2::Tracking::StereoInitialization()’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:526:41: warning: implicitly-declared ‘ORB_SLAM2::Frame& ORB_SLAM2::Frame::operator=(const ORB_SLAM2::Frame&)’ is deprecated [-Wdeprecated-copy]
  526 |         mLastFrame = Frame(mCurrentFrame);
      |                                         ^
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:29,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/FrameDrawer.h:25,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Tracking.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:22:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Frame.h:49:5: note: because ‘ORB_SLAM2::Frame’ has user-provided ‘ORB_SLAM2::Frame::Frame(const ORB_SLAM2::Frame&)’
   49 |     Frame(const Frame &frame);
      |     ^~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc: In member function ‘void ORB_SLAM2::Tracking::MonocularInitialization()’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:553:48: warning: implicitly-declared ‘ORB_SLAM2::Frame& ORB_SLAM2::Frame::operator=(const ORB_SLAM2::Frame&)’ is deprecated [-Wdeprecated-copy]
  553 |             mInitialFrame = Frame(mCurrentFrame);
      |                                                ^
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:29,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/FrameDrawer.h:25,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Tracking.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:22:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Frame.h:49:5: note: because ‘ORB_SLAM2::Frame’ has user-provided ‘ORB_SLAM2::Frame::Frame(const ORB_SLAM2::Frame&)’
   49 |     Frame(const Frame &frame);
      |     ^~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:554:45: warning: implicitly-declared ‘ORB_SLAM2::Frame& ORB_SLAM2::Frame::operator=(const ORB_SLAM2::Frame&)’ is deprecated [-Wdeprecated-copy]
  554 |             mLastFrame = Frame(mCurrentFrame);
      |                                             ^
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:29,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/FrameDrawer.h:25,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Tracking.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:22:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Frame.h:49:5: note: because ‘ORB_SLAM2::Frame’ has user-provided ‘ORB_SLAM2::Frame::Frame(const ORB_SLAM2::Frame&)’
   49 |     Frame(const Frame &frame);
      |     ^~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc: In member function ‘void ORB_SLAM2::Tracking::CreateInitialMapMonocular()’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:710:37: warning: implicitly-declared ‘ORB_SLAM2::Frame& ORB_SLAM2::Frame::operator=(const ORB_SLAM2::Frame&)’ is deprecated [-Wdeprecated-copy]
  710 |     mLastFrame = Frame(mCurrentFrame);
      |                                     ^
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/KeyFrame.h:29,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/MapPoint.h:24,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/FrameDrawer.h:25,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Tracking.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Tracking.cc:22:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/include/Frame.h:49:5: note: because ‘ORB_SLAM2::Frame’ has user-provided ‘ORB_SLAM2::Frame::Frame(const ORB_SLAM2::Frame&)’
   49 |     Frame(const Frame &frame);
      |     ^~~~~
In file included from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.h:229,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/solver.h:32,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/block_solver.h:30,
                 from /home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/src/Optimizer.cc:23:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp: In member function ‘g2o::SparseBlockMatrix<MatrixType>* g2o::SparseBlockMatrix<MatrixType>::slice(int, int, int, int, bool) const’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:328:9: warning: ISO C++ forbids variable length array ‘rowIdx’ [-Wvla]
  328 |     int rowIdx [m];
      |         ^~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:334:9: warning: ISO C++ forbids variable length array ‘colIdx’ [-Wvla]
  334 |     int colIdx [n];
      |         ^~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp: In member function ‘bool g2o::SparseBlockMatrix<MatrixType>::symmPermutation(g2o::SparseBlockMatrix<MatrixType>*&, const int*, bool) const’:
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:404:9: warning: ISO C++ forbids variable length array ‘blockSizes’ [-Wvla]
  404 |     int blockSizes[_rowBlockIndices.size()];
      |         ^~~~~~~~~~
/home/li/Lab/src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp:410:9: warning: ISO C++ forbids variable length array ‘pBlockIndices’ [-Wvla]
  410 |     int pBlockIndices[_rowBlockIndices.size()];
      |         ^~~~~~~~~~~~~
gmake[2]: *** [CMakeFiles/orb_slam2_ros_core.dir/build.make:258: CMakeFiles/orb_slam2_ros_core.dir/orb_slam2/src/Frame.cc.o] Error 1
gmake[2]: *** [CMakeFiles/orb_slam2_ros_core.dir/build.make:202: CMakeFiles/orb_slam2_ros_core.dir/orb_slam2/src/KeyFrame.cc.o] Error 1
gmake[2]: *** [CMakeFiles/orb_slam2_ros_core.dir/build.make:104: CMakeFiles/orb_slam2_ros_core.dir/orb_slam2/src/LocalMapping.cc.o] Error 1
gmake[2]: *** [CMakeFiles/orb_slam2_ros_core.dir/build.make:160: CMakeFiles/orb_slam2_ros_core.dir/orb_slam2/src/FrameDrawer.cc.o] Error 1
gmake[2]: *** [CMakeFiles/orb_slam2_ros_core.dir/build.make:90: CMakeFiles/orb_slam2_ros_core.dir/orb_slam2/src/Tracking.cc.o] Error 1
gmake[2]: *** [CMakeFiles/orb_slam2_ros_core.dir/build.make:118: CMakeFiles/orb_slam2_ros_core.dir/orb_slam2/src/LoopClosing.cc.o] Error 1
gmake[2]: *** [CMakeFiles/orb_slam2_ros_core.dir/build.make:76: CMakeFiles/orb_slam2_ros_core.dir/orb_slam2/src/System.cc.o] Error 1
gmake[2]: *** [CMakeFiles/orb_slam2_ros_core.dir/build.make:230: CMakeFiles/orb_slam2_ros_core.dir/orb_slam2/src/Optimizer.cc.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:651: CMakeFiles/orb_slam2_ros_core.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< orb_slam2_ros [2min 11s, exited with code 2]

Summary: 0 packages finished [2min 12s]
  1 package failed: orb_slam2_ros
  1 package had stderr output: orb_slam2_ros
```