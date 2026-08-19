#!/usr/bin/env bash
set -euo pipefail

if [[ ! -r /etc/os-release ]]; then
  echo "Cannot determine the operating system." >&2
  exit 1
fi

source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_CODENAME:-}" != "noble" ]]; then
  echo "This installer supports Ubuntu 24.04 (Noble) only." >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y curl software-properties-common
sudo add-apt-repository -y universe

ros_apt_source_version="$({
  curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | sed -n 's/.*"tag_name": "\([^"]*\)".*/\1/p'
})"

if [[ -z "${ros_apt_source_version}" ]]; then
  echo "Could not determine the latest ros2-apt-source release." >&2
  exit 1
fi

ros_apt_deb="/tmp/ros2-apt-source-${ros_apt_source_version}.deb"
curl -fsSL -o "${ros_apt_deb}" \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.noble_all.deb"
sudo dpkg -i "${ros_apt_deb}"

sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-clearpath-common \
  ros-jazzy-clearpath-control \
  ros-jazzy-clearpath-platform-description \
  ros-jazzy-clearpath-manipulators-description \
  ros-jazzy-ur-description \
  ros-jazzy-controller-manager \
  ros-jazzy-diff-drive-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-dev-tools

if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

echo
echo "ROS 2 Jazzy installation complete."
echo "Open a new terminal, or run: source /opt/ros/jazzy/setup.bash"

