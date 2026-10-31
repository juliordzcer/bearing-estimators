# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.8)
project(uav_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED CONFIG)
find_package(uav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${uav_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)

# PID Bearing-Only Vision
add_executable(pid_bearing_only_vision
  src/pid.cpp
  src/pid_bearing.cpp
  src/pid_bearing_only_vision.cpp
)

target_link_libraries(pid_bearing_only_vision PUBLIC
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  Eigen3::Eigen
  ${uav_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

install(TARGETS
  pid_bearing_only_vision
  DESTINATION lib/${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
)

ament_export_dependencies(
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  Eigen3
  uav_msgs
  visualization_msgs
)

ament_package()
