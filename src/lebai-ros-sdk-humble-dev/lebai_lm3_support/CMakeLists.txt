cmake_minimum_required(VERSION 3.8)
project(lebai_lm3_support)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)


install(DIRECTORY launch meshes config rviz urdf DESTINATION
    share/${PROJECT_NAME}
)

# 把本包的 share 目录加进 GAZEBO_MODEL_PATH:
# URDF 里的 package://lebai_lm3_support/meshes/... 转 SDF 后会变成
# model://lebai_lm3_support/..., gzserver 需要能在 GAZEBO_MODEL_PATH 里找到它,
# 否则会去请求已停运的在线模型库(models.gazebosim.org)并卡死模型加载。
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
