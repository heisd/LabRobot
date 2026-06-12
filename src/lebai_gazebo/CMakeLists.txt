cmake_minimum_required(VERSION 3.8)
project(lebai_gazebo)

find_package(ament_cmake REQUIRED)

# 仅安装资源文件(urdf / world / config / launch / 自定义模型, 含 ArUco 标记模型)
install(DIRECTORY
  urdf
  worlds
  config
  launch
  models
  DESTINATION share/${PROJECT_NAME}
)

# ArUco 标记生成脚本(可执行)
install(PROGRAMS
  scripts/make_aruco_marker.py
  DESTINATION lib/${PROJECT_NAME}
)

# 把本包 models 目录加进 GAZEBO_MODEL_PATH, 让 world 里的 model://aruco_marker 能被解析
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
