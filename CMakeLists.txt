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
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Optional hardware-native RealSense C++ SDK
find_package(realsense2 QUIET)
if(realsense2_FOUND)
  message(STATUS "Found librealsense2. Enabling direct hardware streaming API.")
  add_compile_definitions(HAVE_REALSENSE2)
else()
  message(STATUS "librealsense2 not found. Direct hardware API disabled; ROS 2 Depth subscription will be used.")
endif()

# Build C++ / CUDA Tracker Node
add_executable(cloth_tracker_cuda_node
  src/cloth_tracker_cuda_node.cpp
  src/ballistic_tracker.cpp
  src/map_background.cpp
  src/map_tracking_control.cpp
  src/cuda_kernels.cu
)

target_include_directories(cloth_tracker_cuda_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CUDAToolkit_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(cloth_tracker_cuda_node CUDA::cudart ${OpenCV_LIBRARIES})

ament_target_dependencies(cloth_tracker_cuda_node
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2
  tf2_ros
  cv_bridge
)

if(realsense2_FOUND)
  target_link_libraries(cloth_tracker_cuda_node ${realsense2_LIBRARY} CUDA::cudart)
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
  realsense_dust_cloth/sim_cloth_flight.py
  realsense_dust_cloth/decompress_image.py
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
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(map_background_test test/test_map_background.cu src/map_background.cpp)
  target_include_directories(map_background_test PRIVATE include ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(map_background_test CUDA::cudart ${OpenCV_LIBRARIES})
  target_compile_options(map_background_test PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-UNDEBUG>)
  add_test(NAME map_background COMMAND map_background_test)
  set_tests_properties(map_background PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(observed_landing_test test/test_observed_landing.cpp src/ballistic_tracker.cpp)
  target_include_directories(observed_landing_test PRIVATE include ${CUDAToolkit_INCLUDE_DIRS})
  target_link_libraries(observed_landing_test CUDA::cudart)
  target_compile_options(observed_landing_test PRIVATE -UNDEBUG)
  add_test(NAME observed_landing COMMAND observed_landing_test)
endif()

ament_package()
