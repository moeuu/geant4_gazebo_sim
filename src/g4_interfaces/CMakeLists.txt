cmake_minimum_required(VERSION 3.16)
project(g4_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

set(MSG_FILES
  msg/ControlEvent.msg
  msg/MeasurementRequest.msg
  msg/MeasurementResult.msg
)

set(SRV_FILES
  srv/Control.srv
  srv/Measurement.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
