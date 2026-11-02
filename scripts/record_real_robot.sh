#!/usr/bin/env bash
# ==============================================================================
# record_real_robot.sh
# Real Robot Rosbag Recording Script for RealSense Cloth Tracker & Localization
# ==============================================================================
set -e

DATE_STR=$(date +%Y%m%d_%H%M%S)
OUTPUT_DIR="${1:-$HOME/rosbag/real_robot_${DATE_STR}}"
mkdir -p "$(dirname "$OUTPUT_DIR")"

echo "======================================================================"
echo " Starting Real Robot Rosbag Recording"
echo " Destination: ${OUTPUT_DIR}"
echo " Press Ctrl+C when finished to save and close the rosbag."
echo "======================================================================"

ros2 bag record \
  -o "${OUTPUT_DIR}" \
  /tf \
  /tf_static \
  /platform_constraint \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \
  /camera/camera/color/image_raw/compressed \
  /camera/camera/color/camera_info \
  /robot/operation/cmd_vel

