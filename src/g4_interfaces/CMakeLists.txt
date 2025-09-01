cmake_minimum_required(VERSION 3.16)
project(g4_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(MSG_FILES
  msg/ControlEvent.msg
)

set(SRV_FILES
  srv/Control.srv
)

# ★ rosidl_generate_interfaces は ament_package() より前に書く
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
