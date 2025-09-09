
cmake_minimum_required(VERSION 3.5)
project(mir_actions)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mir_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  action/MirMoveBase.action
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs mir_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
