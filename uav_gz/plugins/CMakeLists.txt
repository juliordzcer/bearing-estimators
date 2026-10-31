# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
# NOTE: This file is no longer used for building.
#
# The three Gazebo plugins (SetPosePlugin, RobotPosePublisher,
# RobotVelocityPublisher) are now compiled and installed directly by
# the parent package's CMakeLists.txt (uav_gz/CMakeLists.txt).
#
# Plugins are installed to:
#   install/uav_gz/lib/uav_gz/
#
# GZ_SIM_SYSTEM_PLUGIN_PATH is set automatically via ament env-hooks
# after sourcing install/setup.bash — no manual export needed.
#
# This file is kept only as a reference for the standalone build
# procedure. Do NOT run cmake/make directly in this directory.
