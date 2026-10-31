#!/usr/bin/env bash
# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
#
# Installs everything this repository needs on Ubuntu 24.04:
# ROS 2 Jazzy, Gazebo Harmonic, the ROS packages and the Python packages.
# Run it from the repository folder: ./install_deps.sh
set -euo pipefail

if [[ "$(. /etc/os-release && echo "$VERSION_ID")" != "24.04" ]]; then
  echo "This script supports Ubuntu 24.04 only (ROS 2 Jazzy needs it)." >&2
  exit 1
fi

SUDO=""
[[ "$(id -u)" -ne 0 ]] && SUDO="sudo"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "==> System packages"
$SUDO apt-get update
$SUDO apt-get install -y curl lsb-release python3-pip python3-tk

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "==> ROS 2 Jazzy"
  $SUDO apt-get install -y software-properties-common
  $SUDO add-apt-repository -y universe
  VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | cut -d'"' -f4)
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VERSION}/ros2-apt-source_${VERSION}.noble_all.deb"
  $SUDO apt-get install -y /tmp/ros2-apt-source.deb
  $SUDO apt-get update
  $SUDO apt-get install -y ros-jazzy-desktop
fi

echo "==> ROS packages used by this repository (includes Gazebo Harmonic)"
$SUDO apt-get install -y \
  python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-ros-gz ros-jazzy-cv-bridge ros-jazzy-image-transport \
  ros-jazzy-tf2-geometry-msgs \
  libeigen3-dev libopencv-dev nlohmann-json3-dev

echo "==> Python packages"
# Ubuntu ships an older OpenCV without the ArUco detector API used here.
python3 -m pip install --user --break-system-packages -r "$REPO_DIR/requirements.txt"

echo
echo "Done. Next steps are in README.md, section 'Build'."
