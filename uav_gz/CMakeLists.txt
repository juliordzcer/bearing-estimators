# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.5)
project(uav_gz)

find_package(ament_cmake REQUIRED)
find_package(uav_msgs REQUIRED)

# ── Gazebo Harmonic plugins ───────────────────────────────────────────────────
set(GZ_SIM_VER       8)
set(GZ_TRANSPORT_VER 13)
set(GZ_MSGS_VER      10)
set(GZ_PLUGIN_VER    2)
set(GZ_MATH_VER      7)
set(GZ_COMMON_VER    5)

find_package(gz-cmake3         QUIET)
find_package(gz-sim${GZ_SIM_VER}           QUIET)
find_package(gz-transport${GZ_TRANSPORT_VER} QUIET)
find_package(gz-msgs${GZ_MSGS_VER}         QUIET)
find_package(gz-plugin${GZ_PLUGIN_VER}     QUIET)
find_package(gz-math${GZ_MATH_VER}         QUIET)
find_package(gz-common${GZ_COMMON_VER}     QUIET)

if(gz-sim${GZ_SIM_VER}_FOUND AND gz-cmake3_FOUND)
  message(STATUS "Gazebo Harmonic found — building plugins")

  # --- SetPosePlugin ---
  add_library(SetPosePlugin SHARED plugins/SetPosePlugin.cc)
  target_include_directories(SetPosePlugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins
  )
  target_link_libraries(SetPosePlugin
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  )

  # --- RobotPosePublisher ---
  add_library(RobotPosePublisher SHARED plugins/RobotPosePublisher.cc)
  target_include_directories(RobotPosePublisher PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins
  )
  target_link_libraries(RobotPosePublisher
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  )

  # --- RobotVelocityPublisher ---
  add_library(RobotVelocityPublisher SHARED plugins/RobotVelocityPublisher.cc)
  target_include_directories(RobotVelocityPublisher PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins
  )
  target_link_libraries(RobotVelocityPublisher
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  )

  install(TARGETS SetPosePlugin RobotPosePublisher RobotVelocityPublisher
    DESTINATION lib/${PROJECT_NAME}
  )

  # Automatic environment setup via ament hooks (no manual .bashrc exports needed)
  ament_environment_hooks(
    "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/50.uav_gz.dsv.in"
  )
else()
  message(WARNING "Gazebo Harmonic not found — plugins will not be built")
endif()

# ── Models, worlds and scripts ────────────────────────────────────────────────
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Exclude ament_uncrustify from auto-detection: .cc/.hh extensions are
  # mapped to '-l C' by ament_uncrustify, but the plugins use C++ syntax
  # (class inheritance) that causes uncrustify to crash with exit 70.
  set(AMENT_LINT_AUTO_EXCLUDE ament_uncrustify)
endif()

ament_package()
