cmake_minimum_required(VERSION 3.8)
project(robot_mapping)

# Enable strict compilation warnings for supported compilers
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# Add additional dependencies if required
# find_package(<dependency> REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  ${PROJECT_NAME}/mapping_with_known_poses.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the maps directory (contents only)
install(
  DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

# Install additional directories such as config and launch
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Enable testing if BUILD_TESTING is ON
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Skip linter checks for copyrights (remove when adding copyright/license)
  set(ament_cmake_copyright_FOUND TRUE)

  # Skip cpplint (only for Git repositories; remove for production use)
  set(ament_cmake_cpplint_FOUND TRUE)

  # Automatically find and add lint test dependencies
  ament_lint_auto_find_test_dependencies()
endif()

# Add dependencies for the package
ament_export_dependencies(pointcloud_to_laserscan) # Export pointcloud_to_laserscan

# Finalize the package
ament_package()
