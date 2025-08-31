cmake_minimum_required(VERSION 3.8)
project(rangerlow_proto)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/RangerProto.msg
  DEPENDENCIES std_msgs
)

# Install Python executables
install(PROGRAMS
  scripts/slip_driver_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()