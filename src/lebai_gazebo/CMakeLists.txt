cmake_minimum_required(VERSION 3.8)
project(lebai_gazebo)

find_package(ament_cmake REQUIRED)

# 仅安装资源文件(urdf / world / config / launch)
install(DIRECTORY
  urdf
  worlds
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
