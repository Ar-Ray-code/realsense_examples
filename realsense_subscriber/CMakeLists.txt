cmake_minimum_required(VERSION 3.5)
project(realsense_subscriber)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenCV REQUIRED)

set(OpenCV_INCLUDE_DIRS)
set(OpenCV_LIB_DIR)

#------------------------------------------------

include_directories(realsense_sub
  ${OpenCV_INCLUDE_DIRS}
)
include_directories(realsense_depth
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(realsense_sub
  src/realsense_sub.cpp
)
add_executable(realsense_depth
  src/realsense_sub_depth.cpp
)

ament_target_dependencies(realsense_sub
  rclcpp
  image_transport
  cv_bridge
  std_msgs
  sensor_msgs
)
ament_target_dependencies(realsense_depth
  rclcpp
  image_transport
  cv_bridge
  std_msgs
  sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  realsense_sub
  realsense_depth
  DESTINATION lib/${PROJECT_NAME}
)


#--------------------------------------------------
ament_package()