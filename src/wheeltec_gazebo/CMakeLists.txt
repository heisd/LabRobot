cmake_minimum_required(VERSION 3.8)
project(wheeltec_gazebo)

find_package(ament_cmake REQUIRED)

# 仅安装资源文件(urdf / worlds / launch / 自定义 Gazebo 模型, 含 QR 码模型)
install(DIRECTORY
  urdf
  worlds
  launch
  models
  DESTINATION share/${PROJECT_NAME}
)

# 把本包的 models 目录加进 GAZEBO_MODEL_PATH, 让 world 里的 model://qr_* 能被解析
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
