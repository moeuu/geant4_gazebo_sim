cmake_minimum_required(VERSION 3.16)
project(g4_bringup)

find_package(ament_cmake REQUIRED)

# launch / robots / worlds / resource を share にインストール
install(DIRECTORY
  launch
  robots
  worlds
  resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
