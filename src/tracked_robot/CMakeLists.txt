cmake_minimum_required(VERSION 3.8)
project(tracked_robot)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tracked_robot_msgs REQUIRED)

# Create directories
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/scripts)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/maps)

# Install directories
install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install script directories conditionally
install(
  DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*"
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
             GROUP_READ GROUP_EXECUTE
             WORLD_READ WORLD_EXECUTE
)

# Install maps directory
install(
  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install Python scripts
install(PROGRAMS
  scripts/odrive_control_node.py
  scripts/auto_docking_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
