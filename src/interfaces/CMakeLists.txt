cmake_minimum_required(VERSION 3.8)
project(interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "buff_msg/Buff.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)
ament_export_dependencies(interfaces)


ament_package()
