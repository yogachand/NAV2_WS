cmake_minimum_required(VERSION 3.8)
project(robot_safety_features)

# Compiler options for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) # Required for Twist and TwistStamped messages

# Install Python scripts specific to robot_safety_features
install(
  PROGRAMS
    ${CMAKE_CURRENT_SOURCE_DIR}/robot_safety_features/safety_stop.py
    ${CMAKE_CURRENT_SOURCE_DIR}/robot_safety_features/twist_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (avoid nested launch directories)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install configuration files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Declare this as an ament package
ament_package()
