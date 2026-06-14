cmake_minimum_required(VERSION 3.5)
project(bodyreader)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Architecture Detection
execute_process(COMMAND uname -m OUTPUT_VARIABLE MACHINES)
if (${MACHINES} MATCHES "x86_64")
  set(HOST_PLATFORM "x64")
else()
  set(HOST_PLATFORM "arm64")
endif ()
message(STATUS "Architecture: ${MACHINES}, using lib/${HOST_PLATFORM}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(bodyreader_msg REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)  
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
)

# Only link SDK libraries if they exist (for ARM compatibility)
# On x86_64, we use the MediaPipe Python node instead of C++ main
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/${HOST_PLATFORM}/libastra.so")
    link_directories(
      lib/${HOST_PLATFORM} 
      lib/${HOST_PLATFORM}/Plugins 
      lib/${HOST_PLATFORM}/Plugins/openni2 
      lib/${HOST_PLATFORM}/Plugins/obt/3rdparty/lib
    )
    
    # We only build the C++ nodes that need the SDK if the SDK is present
    add_executable(main_cpp src/main.cpp)
    ament_target_dependencies(main_cpp rclcpp bodyreader_msg sensor_msgs)
    target_link_libraries(main_cpp astra astra_core astra_core_api)
    
    add_executable(bodydata_process src/bodydata_process.cpp)
    ament_target_dependencies(bodydata_process rclcpp bodyreader_msg sensor_msgs std_msgs geometry_msgs)
    target_link_libraries(bodydata_process astra astra_core astra_core_api)

    add_executable(follower src/follower.cpp)
    ament_target_dependencies(follower rclcpp bodyreader_msg sensor_msgs std_msgs geometry_msgs)
    target_link_libraries(follower astra astra_core astra_core_api)

    add_executable(interaction src/interaction.cpp)
    ament_target_dependencies(interaction rclcpp bodyreader_msg geometry_msgs)
    target_link_libraries(interaction astra astra_core astra_core_api)

    add_executable(feedback src/feedback.cpp)
    ament_target_dependencies(feedback rclcpp bodyreader_msg geometry_msgs)
    target_link_libraries(feedback astra astra_core astra_core_api)

    add_executable(image_trans src/image_trans.cpp)
    ament_target_dependencies(image_trans rclcpp bodyreader_msg sensor_msgs image_transport)
    target_link_libraries(image_trans astra astra_core astra_core_api)
else()
    message(WARNING "Orbbec SDK not found for ${HOST_PLATFORM}. C++ nodes requiring SDK will not be built.")
    # On x64 without SDK, we can still build nodes that DON'T need the SDK 
    # (though in this package most do). 
    # For now, we rely on the MediaPipe Python replacement for 'main'.
endif()

# Install Python scripts (including the new MediaPipe replacement for 'main')
install(PROGRAMS 
    src/mediapipe_bodyreader.py 
    src/mediapipe_body_process.py
    scripts/display.py 
    scripts/compressed.py 
    DESTINATION lib/${PROJECT_NAME}
)

# Rename the python script to 'main' in the install directory to match launch files
# Or just handle it in launch. For now, we'll keep it as mediapipe_bodyreader.py
# and modify the launch file or use an alias. 

install(
  DIRECTORY launch audio
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/${HOST_PLATFORM}/")
    install(DIRECTORY lib/${HOST_PLATFORM}/
      DESTINATION lib/
    )
endif()

ament_package()

