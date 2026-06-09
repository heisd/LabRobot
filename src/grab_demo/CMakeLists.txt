cmake_minimum_required(VERSION 3.8)
project(grab_demo)

# 如果没有指定构建类型，默认为 Debug
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelwithDebInfo)

endif()

# Debug 模式下的编译选项
# set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

set(CMAKE_CXX_STANDARD 11)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(moveit_common REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)  # 添加这一行
find_package(pcl_ros REQUIRED)          # 添加这一行
find_package(lebai_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

moveit_package()
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GrabObject.srv"
)
ament_export_dependencies(rosidl_default_runtime)





add_executable(charuco_dectet_node src/charuco_dectet_node.cpp)
ament_target_dependencies(charuco_dectet_node
 rclcpp
 Eigen3
 image_transport
 geometry_msgs
 sensor_msgs
 cv_bridge
 OpenCV
 moveit_ros_planning_interface
)
install(TARGETS charuco_dectet_node
  DESTINATION lib/${PROJECT_NAME})

  
add_executable(hand_eye src/hand_eye.cpp)
ament_target_dependencies(hand_eye
 rclcpp
 Eigen3
 image_transport
 geometry_msgs
 sensor_msgs
 cv_bridge
 OpenCV
 moveit_ros_planning_interface
 moveit_core
 tf2_ros
 tf2_geometry_msgs
 tf2
)
install(TARGETS hand_eye
  DESTINATION lib/${PROJECT_NAME})



add_executable(hsv_range src/hsv_range.cpp)
ament_target_dependencies(hsv_range
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
  message_filters
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS hsv_range
  DESTINATION lib/${PROJECT_NAME})




  add_executable(aruco_dectet src/aruco_dectet.cpp)
  ament_target_dependencies(aruco_dectet
    rclcpp
    Eigen3
    image_transport
    geometry_msgs
    sensor_msgs
    cv_bridge
    OpenCV
    message_filters
    tf2_geometry_msgs
  )
  install(TARGETS aruco_dectet
    DESTINATION lib/${PROJECT_NAME})  






  add_executable(grab_service_node src/grab_service_node.cpp)
  ament_target_dependencies(grab_service_node
    rclcpp
    geometry_msgs
    tf2_geometry_msgs
    tf2_ros
    moveit_ros_planning_interface
    lebai_interfaces
  )
  install(TARGETS grab_service_node
    DESTINATION lib/${PROJECT_NAME})

rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(grab_service_node "${cpp_typesupport_target}")



# ---------------------------------------------------------------------------
# 闭环 (PBVS) 抓取服务节点: grab_service_node 的"看-动-再看-修正"闭环版,
# 对外接口完全一致(同样订阅 target_frame / 提供 obj_grab_service)。
# ---------------------------------------------------------------------------
add_executable(closed_loop_grab_node src/closed_loop_grab_node.cpp)
ament_target_dependencies(closed_loop_grab_node
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  moveit_ros_planning_interface
  lebai_interfaces
)
install(TARGETS closed_loop_grab_node
  DESTINATION lib/${PROJECT_NAME})

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(closed_loop_grab_node "${cpp_typesupport_target}")



add_executable(start_grab src/start_grab.cpp)
ament_target_dependencies(start_grab
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  moveit_ros_planning_interface
  lebai_interfaces
)
install(TARGETS start_grab
  DESTINATION lib/${PROJECT_NAME})

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(start_grab "${cpp_typesupport_target}")




add_executable(camera_info_node src/camera_info_node.cpp)
ament_target_dependencies(camera_info_node
 rclcpp
 sensor_msgs
)
install(TARGETS camera_info_node
  DESTINATION lib/${PROJECT_NAME})


  add_executable(nav_grab src/nav_grab.cpp)
  ament_target_dependencies(nav_grab
   rclcpp
   geometry_msgs
   nav2_msgs
   rclcpp_action
  )
  install(TARGETS nav_grab
    DESTINATION lib/${PROJECT_NAME})
  
    rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp)
  
  target_link_libraries(nav_grab "${cpp_typesupport_target}")
  # 使用AI给我们提供的PCL库来进行操作
  add_executable(point_cloud_node src/point_cloud_node.cpp)
  # 添加依赖
  ament_target_dependencies(point_cloud_node
   rclcpp
   OpenCV
   cv_bridge
   geometry_msgs
   sensor_msgs
   rclcpp_action
   pcl_ros
   pcl_conversions
  )
  # 安装
  install(TARGETS point_cloud_node
    DESTINATION lib/${PROJECT_NAME})




# ---------------------------------------------------------------------------
# YOLO TensorRT 检测节点 (可选)
# 仅在能找到 CUDA + TensorRT 的环境(通常是 Jetson)上编译, 否则自动跳过,
# 不会影响其它节点的正常构建。
# ---------------------------------------------------------------------------
find_package(CUDA QUIET)
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu
        /usr/include /usr/local/tensorrt/include /usr/local/TensorRT/include)
find_library(TENSORRT_NVINFER nvinfer
  HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu
        /usr/local/tensorrt/lib /usr/local/TensorRT/lib)
find_library(TENSORRT_ONNXPARSER nvonnxparser
  HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu
        /usr/local/tensorrt/lib /usr/local/TensorRT/lib)

if(CUDA_FOUND AND TENSORRT_INCLUDE_DIR AND TENSORRT_NVINFER)
  message(STATUS "找到 CUDA 与 TensorRT, 编译 yolo_detect_node")
  add_executable(yolo_detect_node src/yolo_detect_node.cpp)
  ament_target_dependencies(yolo_detect_node
    rclcpp
    sensor_msgs
    std_msgs
    cv_bridge
    OpenCV
    message_filters
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
  )
  target_include_directories(yolo_detect_node PRIVATE
    ${TENSORRT_INCLUDE_DIR} ${CUDA_INCLUDE_DIRS})
  target_link_libraries(yolo_detect_node
    ${TENSORRT_NVINFER} ${CUDA_LIBRARIES})
  # 本节点使用 std::clamp 等 C++17 特性, 单独为该 target 指定 C++17
  # (不影响包内其它使用 C++11 的目标)
  target_compile_features(yolo_detect_node PRIVATE cxx_std_17)
  # 找到 onnx 解析器时, 允许节点在运行时直接从 .onnx 构建 engine
  if(TENSORRT_ONNXPARSER)
    target_link_libraries(yolo_detect_node ${TENSORRT_ONNXPARSER})
    target_compile_definitions(yolo_detect_node PRIVATE HAVE_ONNX_PARSER=1)
  endif()
  install(TARGETS yolo_detect_node
    DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "未找到 CUDA 或 TensorRT, 跳过 yolo_detect_node 的编译 (仅 Jetson/带GPU环境需要)")
endif()

# ---------------------------------------------------------------------------
# KCF 跟踪节点 (可选)
# 需要 opencv_contrib 的 tracking 模块(提供 cv::TrackerKCF, opencv2/tracking.hpp)。
# 找不到该模块时自动跳过编译, 不影响其它节点。
# ---------------------------------------------------------------------------
find_path(OPENCV_TRACKING_HEADER opencv2/tracking.hpp
  HINTS ${OpenCV_INCLUDE_DIRS} /usr/include/opencv4 /usr/local/include/opencv4
        /usr/include /usr/local/include)
if(OPENCV_TRACKING_HEADER)
  message(STATUS "找到 OpenCV tracking 模块, 编译 kcf_track_node")
  add_executable(kcf_track_node src/kcf_track_node.cpp)
  ament_target_dependencies(kcf_track_node
    rclcpp
    sensor_msgs
    std_srvs
    std_msgs
    cv_bridge
    OpenCV
    message_filters
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
  )
  install(TARGETS kcf_track_node
    DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "未找到 OpenCV tracking 模块(opencv_contrib), 跳过 kcf_track_node 的编译")
endif()

# VLM(视觉语言模型)抓取节点 —— 纯 Python, 用 ros2 run grab_demo vlm_grab_node.py 运行
# yolo_ros 桥接节点 —— 纯 Python, 把官方 yolo_ros 的检测接到统一 target_frame
install(PROGRAMS
  scripts/vlm_grab_node.py
  scripts/yolo_ros_detect_node.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
