cmake_minimum_required(VERSION 3.16)
project(geant4_embed LANGUAGES CXX)

# ament + rclcpp（ROS 2 Jazzy）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Geant4
find_package(Geant4 REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
  ${Geant4_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_executable(geant4_embed_node
  src/main.cpp
  src/sim_shared.cpp
  src/odom_subscriber.cpp
  src/geant4/DetectorConstruction.cc
  src/geant4/ActionInitialization.cc
  src/geant4/PrimaryGeneratorAction.cc
  src/geant4/EventAction.cc
)

target_link_libraries(geant4_embed_node
  ${Geant4_LIBRARIES}
)

ament_target_dependencies(geant4_embed_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)

install(TARGETS geant4_embed_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
