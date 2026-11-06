cmake_minimum_required(VERSION 3.18)
project(realsense_dust_cloth LANGUAGES CXX CUDA)

if(NOT CMAKE_CUDA_ARCHITECTURES)
  # Jetson Nano (sm_53), Xavier (sm_72), RTX 3050 (sm_86), Orin (sm_87)
  set(CMAKE_CUDA_ARCHITECTURES 53 72 86 87)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

find_package(CUDAToolkit REQUIRED)
# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

# Optional hardware-native RealSense C++ SDK
find_package(realsense2 QUIET)
if(realsense2_FOUND)
  message(STATUS "Found librealsense2. Enabling direct hardware streaming API.")
  add_compile_definitions(HAVE_REALSENSE2)
else()
  message(STATUS "librealsense2 not found. Direct mode is unavailable; topic input requires the Unity/rosbag profile.")
endif()

# Build C++ / CUDA Tracker Node
set(TRACKER_SOURCES
  src/cloth_tracker_cuda_node.cpp
  src/tracker_parameters.cpp
  src/camera_input.cpp
  src/realsense_camera.cpp
  src/color_stream.cpp
  src/tracking_output.cpp
  src/ballistic_tracker.cpp
  src/tracking_status.cpp
  src/external_cloud_tracking.cpp
  src/cuda_kernels.cu
)

add_library(cloth_tracker_core STATIC ${TRACKER_SOURCES})
add_executable(cloth_tracker_cuda_node src/main.cpp)
target_link_libraries(cloth_tracker_cuda_node cloth_tracker_core)

target_include_directories(cloth_tracker_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CUDAToolkit_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(cloth_tracker_core CUDA::cudart ${OpenCV_LIBRARIES})

ament_target_dependencies(cloth_tracker_core
  ament_index_cpp
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2
  tf2_ros
)

if(realsense2_FOUND)
  target_link_libraries(cloth_tracker_core ${realsense2_LIBRARY} CUDA::cudart)
endif()

# Install C++ executable
install(TARGETS cloth_tracker_cuda_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules and scripts
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  realsense_dust_cloth/trajectory_probability_analyzer.py
  realsense_dust_cloth/record_trajectories.py
  realsense_dust_cloth/decompress_image.py
  realsense_dust_cloth/param_tuner_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  rviz
  data
  scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME launch_profiles COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch_profiles.py)
  add_executable(point_cloud_io_test test/test_point_cloud_io.cpp)
  target_include_directories(point_cloud_io_test PRIVATE include ${CUDAToolkit_INCLUDE_DIRS})
  ament_target_dependencies(point_cloud_io_test sensor_msgs)
  target_compile_options(point_cloud_io_test PRIVATE -UNDEBUG)
  add_test(NAME point_cloud_io COMMAND point_cloud_io_test)
  add_executable(observed_landing_test test/test_observed_landing.cpp src/ballistic_tracker.cpp)
  target_include_directories(observed_landing_test PRIVATE include ${CUDAToolkit_INCLUDE_DIRS})
  target_link_libraries(observed_landing_test CUDA::cudart)
  target_compile_options(observed_landing_test PRIVATE -UNDEBUG)
  add_test(NAME observed_landing COMMAND observed_landing_test)
  add_executable(tracking_recovery_test test/test_tracking_recovery.cpp src/ballistic_tracker.cpp)
  target_include_directories(tracking_recovery_test PRIVATE include ${CUDAToolkit_INCLUDE_DIRS})
  target_link_libraries(tracking_recovery_test CUDA::cudart)
  target_compile_options(tracking_recovery_test PRIVATE -UNDEBUG)
  add_test(NAME tracking_recovery COMMAND tracking_recovery_test)
  add_executable(runtime_pipeline_test test/test_runtime_pipeline.cpp)
  target_link_libraries(runtime_pipeline_test cloth_tracker_core)
  target_compile_options(runtime_pipeline_test PRIVATE -UNDEBUG)
  add_test(NAME runtime_pipeline COMMAND runtime_pipeline_test ${CMAKE_CURRENT_SOURCE_DIR}/config/tracker_params.yaml)
  set_tests_properties(runtime_pipeline PROPERTIES SKIP_RETURN_CODE 77 ENVIRONMENT "ROS_DOMAIN_ID=96;ROS_LOCALHOST_ONLY=1")
endif()

ament_package()
