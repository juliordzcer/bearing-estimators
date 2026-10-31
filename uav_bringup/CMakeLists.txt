# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.8)
project(uav_bringup)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Find Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(uav_msgs REQUIRED) 
find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED CONFIG)

# =========================================
# Executables & Targets (Modern CMake Style)
# =========================================

# --- Node: set_pose ---
add_executable(set_pose src/set_pose.cpp)
target_link_libraries(set_pose PUBLIC
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  nlohmann_json::nlohmann_json
)

install(TARGETS set_pose DESTINATION lib/${PROJECT_NAME})

# --- Node: wind_node ---
add_executable(wind_node src/wind.cpp)
target_link_libraries(wind_node PUBLIC
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

install(TARGETS wind_node DESTINATION lib/${PROJECT_NAME})


# =========================================
# Install Resources
# =========================================

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()