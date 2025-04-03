cmake_minimum_required(VERSION 3.8)
project(minimal_tracked_robot)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Create output directories
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/urdf)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/launch)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/config)

# Copy the URDF files
foreach(file
  tracked_robot_properties.xacro
  tracked_chassis.xacro
  zed_camera.xacro
  tracked_robot_transmissions.xacro
  tracked_robot_gazebo.xacro
  tracked_robot.urdf.xacro
)
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/../tracked_robot/urdf/${file}
    ${CMAKE_CURRENT_BINARY_DIR}/urdf/${file}
    COPYONLY
  )
endforeach()

# Copy our simple robot URDF which doesn't depend on the tracked_robot package
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/urdf/simple_robot.xacro
  ${CMAKE_CURRENT_BINARY_DIR}/urdf/simple_robot.xacro
  COPYONLY
)

# Copy the config files
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/../tracked_robot/config/robot.rviz
  ${CMAKE_CURRENT_BINARY_DIR}/config/robot.rviz
  COPYONLY
)

# Copy launch files
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/view_robot.launch.py
  ${CMAKE_CURRENT_BINARY_DIR}/launch/view_robot.launch.py
  COPYONLY
)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/simple_robot.launch.py
  ${CMAKE_CURRENT_BINARY_DIR}/launch/simple_robot.launch.py
  COPYONLY
)

# Install directories
install(DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
