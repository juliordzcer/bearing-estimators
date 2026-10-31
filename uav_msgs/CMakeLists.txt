# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.5)
project(uav_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BearingMeasurement.msg"
  "msg/BearingMeasurementsArray.msg"
  "msg/EstimationError.msg"
  "msg/EstimationErrorsArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
