cmake_minimum_required(VERSION 3.8)
project(arm_demo)

# WSL fix: Prevent finding Windows CURL/packages
set(CURL_NO_CURL_CMAKE ON)
set(CMAKE_IGNORE_PREFIX_PATH "/mnt/c")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Common cmake code applied to all moveit packages
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()

add_executable(ik_demo src/ik_demo.cpp)
ament_target_dependencies(ik_demo
  moveit_ros_planning_interface
)

add_executable(fk_demo src/fk_demo.cpp)
target_include_directories(fk_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(fk_demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(fk_demo
  moveit_ros_planning_interface
)

install(TARGETS fk_demo
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS ik_demo
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
