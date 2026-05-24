# wheeltec_rviz2 (Wheeltec 系列机器人的 RViz2 可视化配置包)

## 概述

`wheeltec_rviz2` 是 Wheeltec S300 系列 ROS2 机器人配套的可视化配置包。该包本身不包含任何 C++/Python 节点代码，主要作用是：

1. 提供 Wheeltec 机器人在 RViz2 中常用的预置显示配置文件 (`.rviz`)，包括 2D 导航/SLAM 场景下的主配置和 ORB-SLAM (RGBD) 视觉建图场景下的配置。
2. 提供启动 RViz2 (`rviz2`) 节点并加载这些预置配置的 ROS2 launch 文件。
3. 提供启动 `rtabmap_ros` 中 `rtabmapviz` 可视化工具的 launch 文件，便于 RTAB-Map 三维 SLAM 调试。

该包属于 Wheeltec ROS2 工作空间 (`wheeltec_ros2`) 中的可视化辅助子包，通常被建图、导航、ORB-SLAM 等其他功能包通过 `IncludeLaunchDescription` 或独立启动的方式调用。

## 目录结构

```
wheeltec_rviz2/
├── CMakeLists.txt              # 仅安装 launch/ 与 rviz/ 两个目录到 share/
├── package.xml                 # 包元数据，仅依赖 ament_cmake
├── launch/
│   ├── wheeltec_rviz.launch.py        # 启动 rviz2 并加载 wheeltec.rviz
│   └── wheeltec_rtabmapviz.launch.py  # 启动 rtabmap_ros 的 rtabmapviz
└── rviz/
    ├── wheeltec.rviz                  # 主可视化配置 (Nav2 / SLAM / 多视图)
    └── orb_config.rviz                # ORB-SLAM RGBD 建图可视化配置
```

## 依赖项

`package.xml` 中的依赖如下：

- buildtool 依赖
  - `ament_cmake`
- 测试依赖
  - `ament_lint_auto`
  - `ament_lint_common`

虽然包本身没有声明 `exec_depend`，但其内部 launch 文件实际运行时需要安装以下 ROS2 包，否则启动会失败：

- `rviz2`（被 `wheeltec_rviz.launch.py` 调用）
- `rtabmap_ros`（被 `wheeltec_rtabmapviz.launch.py` 调用，节点 `rtabmapviz`）
- `nav2_rviz_plugins`（`wheeltec.rviz` 中使用了 `nav2_rviz_plugins/Navigation 2` 面板）

## 节点说明

本包不包含自定义可执行节点。它通过 launch 文件启动以下第三方节点：

### 1. `rviz2` (来自 `rviz2` 包)

- 由 `launch/wheeltec_rviz.launch.py` 启动。
- 参数：通过 `-d <rviz_config_file>` 加载 RViz 配置文件，默认指向 `share/wheeltec_rviz2/rviz/wheeltec.rviz`。
- 该节点本身不通过本包发布或订阅自定义话题，而是按 `wheeltec.rviz` 的 Displays 配置订阅以下话题（部分列举）：
  - `/robot_description` (RobotModel 显示)
  - `/scan` (LaserScan 显示)
  - `/mobile_base/sensors/bumper_pointcloud` (PointCloud2)
  - `/map`、`/map_updates` (Map 显示)
  - `/particle_cloud` (AMCL 粒子云)
  - `/global_costmap/costmap`、`/global_costmap/costmap_updates`
  - `/downsampled_costmap`、`/downsampled_costmap_updates`
  - `/plan`、`/local_plan` (路径)
  - `/global_costmap/voxel_marked_cloud`、`/local_costmap/voxel_marked_cloud`
  - `/global_costmap/published_footprint`、`/local_costmap/published_footprint`
  - `/local_costmap/costmap`、`/local_costmap/costmap_updates`
  - `/marker` (MarkerArray)
  - `/intel_realsense_r200_depth/image_raw` (Image)
  - `/intel_realsense_r200_depth/points` (PointCloud2)
  - `/waypoints`、`/_shapes`、`/_shapes_array`
  - `/clicked_point`、`/detected_frontiers`
  - `/filtered_centroids`、`/filtered_centroids_array`
  - `/mapData` (Path)
- 固定参考坐标系 (Fixed Frame)：`map`。

### 2. `rtabmapviz` (来自 `rtabmap_ros` 包)

- 由 `launch/wheeltec_rtabmapviz.launch.py` 启动。
- 关键参数（写在 launch 中）：
  - `queue_size`：消息队列长度，默认 `20`
  - `frame_id`：节点参考坐标系，默认 `camera_link`
  - `use_sim_time`：是否使用仿真时间，默认 `false`，可通过 launch 参数覆盖
  - `subscribe_depth`：是否订阅深度图，默认 `True`
- 重映射（remappings）：
  - `odom`           -> `/odom_combined`
  - `rgb/image`      -> `/camera/color/image_raw`
  - `rgb/camera_info` -> `/camera/color/camera_info`
  - `depth/image`    -> `/camera/depth/image`
- 同时在环境变量中设置 `RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED=1`，使日志按行输出。

## 启动文件

### `launch/wheeltec_rviz.launch.py`

- 作用：启动一个 `rviz2` 进程并加载指定 RViz 配置。
- 关键 launch 参数：
  - `rviz_config`：完整路径，默认为本包安装目录下的 `rviz/wheeltec.rviz`。可在命令行通过 `rviz_config:=...` 覆盖。
- 启动的节点：
  - `package=rviz2`, `executable=rviz2`，输出到屏幕，附加参数 `['-d', <rviz_config>]`。

### `launch/wheeltec_rtabmapviz.launch.py`

- 作用：启动 `rtabmap_ros` 的 `rtabmapviz` 可视化工具，用于 RGBD SLAM 实时观察。
- 关键 launch 参数：
  - `use_sim_time`：默认 `false`。
- 启动的节点：
  - `package=rtabmap_ros`, `executable=rtabmapviz`，附带参数与重映射如「节点说明」所列。

## 消息/服务/动作定义

本包不定义任何 `.msg`、`.srv`、`.action` 接口。

## 参数配置

本包没有 `config/*.yaml` 文件，所有可视化设置都保存在 `rviz/` 目录下的 `.rviz` 文件中。下面对两个 RViz 配置作要点说明：

### `rviz/wheeltec.rviz`

面向 Wheeltec 机器人 2D 导航 + SLAM 的综合可视化布局，主要包含：

- 面板：`Displays`、`Selection`、`Tool Properties`、`Views`、`nav2_rviz_plugins/Navigation 2`（Nav2 面板）。
- Fixed Frame：`map`。
- 主要显示项及其订阅话题：
  - `Grid`：基础网格 (10×10)。
  - `RobotModel`：从 `/robot_description` 加载，展示 `base_link`、`camera_link`、`controller_link`、`front_link`、`laser_link`、`lb_link`、`left_link` 等连杆。
  - `TF`：坐标系树。
  - `LaserScan`：订阅 `/scan`。
  - `PointCloud2`：订阅 `/mobile_base/sensors/bumper_pointcloud`。
  - `Map`：订阅 `/map` 与 `/map_updates`。
  - 分组 `Costmap` 等：包含 `/global_costmap/costmap`、`/downsampled_costmap`、`/local_costmap/costmap`、`/global_costmap/voxel_marked_cloud`、`/local_costmap/voxel_marked_cloud`、`/global_costmap/published_footprint`、`/local_costmap/published_footprint`、`/plan`、`/local_plan` 等。
  - 相机相关：`Image` (`/intel_realsense_r200_depth/image_raw`)、`PointCloud2` (`/intel_realsense_r200_depth/points`)。
  - 路径/标记：`/waypoints`、`/_shapes`、`/_shapes_array`、`/clicked_point`、`/detected_frontiers`、`/filtered_centroids`、`/filtered_centroids_array`、`/mapData`。
- Tools：包含 `MoveCamera`、`Select`、`FocusCamera`、`Measure`、`SetInitialPose`、`PublishPoint`。
- View：`XYOrbit`，目标坐标系 `<Fixed Frame>`。

### `rviz/orb_config.rviz`

面向 RGBD ORB-SLAM 与 octomap 调试的布局，主要包含：

- 面板：`Displays`、`Selection`、`Tool Properties`、`Views`、`Time`。
- Fixed Frame：`map`。
- 主要显示项及其订阅话题：
  - `Image`：`/RGBD/debug_image`，ORB-SLAM 调试图像。
  - `PointCloud2`：`/RGBD/map_points` (地图点)、`/RGBD/cloud_points` (当前帧点云)。
  - `Pose`：`/RGBD/pose`，相机位姿。
  - `Path`：`/orb_slam2_rgbd/trajectory`，SLAM 轨迹。
  - `PointCloud2`：`/octomap_point_cloud_centers`。
  - `MarkerArray`：`/occupied_cells_vis_array`，octomap 占据体素。
  - `Map`：`/projected_map`、`/projected_map_updates`，投影 2D 地图。
  - Tools 中包含 `/initialpose`、`/move_base_simple/goal`、`/clicked_point` 等发布点工具。

## 编译与运行

在 Wheeltec ROS2 工作空间根目录下编译：

```bash
cd ~/wheeltec_ros2     # 或包所在工作空间根目录
colcon build --packages-select wheeltec_rviz2
source install/setup.bash
```

启动主可视化界面（加载 `wheeltec.rviz`）：

```bash
ros2 launch wheeltec_rviz2 wheeltec_rviz.launch.py
```

如需加载 ORB-SLAM 配置：

```bash
ros2 launch wheeltec_rviz2 wheeltec_rviz.launch.py \
    rviz_config:=$(ros2 pkg prefix wheeltec_rviz2)/share/wheeltec_rviz2/rviz/orb_config.rviz
```

启动 RTAB-Map 可视化界面：

```bash
ros2 launch wheeltec_rviz2 wheeltec_rtabmapviz.launch.py
```

若使用仿真时间：

```bash
ros2 launch wheeltec_rviz2 wheeltec_rtabmapviz.launch.py use_sim_time:=true
```

## 使用示例

典型流程（以 2D 导航为例）：

1. 在机器人或仿真终端启动底盘、雷达、定位与 Nav2 等模块（由其它包提供，如 `turn_on_wheeltec_robot`、`wheeltec_nav2`）。
2. 在另一终端执行：
   ```bash
   ros2 launch wheeltec_rviz2 wheeltec_rviz.launch.py
   ```
3. 在 RViz2 中通过左侧 `Displays` 面板检查各话题，使用 `2D Pose Estimate` 工具发布初始位姿到 `/initialpose`，使用右下角的 Nav2 面板下发导航目标。

RGBD SLAM 流程：

1. 启动 RTAB-Map (例如 `wheeltec_robot_rtab`)，并发布 `/odom_combined`、`/camera/color/image_raw`、`/camera/color/camera_info`、`/camera/depth/image`。
2. 启动 `wheeltec_rtabmapviz.launch.py` 即可在 `rtabmapviz` 窗口中查看建图过程。

## 注意事项

- 本包仅提供配置与启动，本身不引入任何代码；若运行 launch 时报错找不到 `rviz2`、`rtabmap_ros`、`nav2_rviz_plugins`，请先 `apt install` 对应的 ROS2 包。
- `wheeltec.rviz` 中相机话题以 `intel_realsense_r200_depth` 为前缀，实际使用 Wheeltec 配套的 Astra/D435 等相机时话题可能不同，需要在 RViz 中手动调整或在配置文件中替换。
- `wheeltec_rtabmapviz.launch.py` 中的话题重映射假定相机驱动发布 `/camera/color/image_raw`、`/camera/color/camera_info`、`/camera/depth/image`，里程计名为 `/odom_combined`；若实际话题不同，请相应修改 `remappings`。
- Fixed Frame 默认是 `map`，若运行时尚未发布 `map -> odom -> base_link` 的 TF，可视化会显示警告，需要等待 SLAM/AMCL 启动后再观察。
- 该包没有声明运行期依赖，CI/打包时 `rosdep` 不会自动安装 `rviz2` 等组件，建议在生产部署时手动检查。
