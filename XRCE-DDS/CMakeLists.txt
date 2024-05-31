cmake_minimum_required(VERSION 3.5)
project(pose_subscriber_pkg)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(pose_subscriber src/pose_subscriber.cpp)
ament_target_dependencies(pose_subscriber rclcpp geometry_msgs serial)

# Install targets
install(TARGETS
  pose_subscriber
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
