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

find_package(lebai_interfaces REQUIRED)

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
