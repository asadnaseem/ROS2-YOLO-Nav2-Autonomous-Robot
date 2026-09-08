#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

if [ ! -f "${ROS_SETUP}" ]; then
  echo "ROS 2 ${ROS_DISTRO_NAME} is not installed under /opt/ros. See README.md."
  exit 1
fi

source "${ROS_SETUP}"
if pgrep -x gzserver >/dev/null 2>&1; then
  echo "Another Gazebo server is already running. Stop the earlier launch with Ctrl+C first."
  exit 1
fi
python3 -c 'import ultralytics, cv2, numpy; assert int(numpy.__version__.split(".")[0]) < 2' 2>/dev/null || {
  echo "Installing YOLO Python packages for the current user..."
  python3 -m pip install --user "setuptools<80" "packaging>=24" \
    "numpy<2" "opencv-python<4.12" ultralytics
}

cd "${PROJECT_DIR}"
colcon build --symlink-install --packages-select yolo_nav2_semantic
source install/setup.bash
ros2 launch yolo_nav2_semantic semantic_nav.launch.py
