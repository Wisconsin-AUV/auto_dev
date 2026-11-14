#!/usr/bin/env bash
# Usage: ./record_bag.sh [label] [storage]
#   ./record_bag.sh trim_v1
#   ./record_bag.sh trim_v1 mcap
# Ctrl+C to stop and close the bag.

LABEL="${1:-}"
STORAGE="${2:-sqlite3}"
BAG_DIR="${BAG_DIR:-$HOME/wauv_bags}"

source /opt/ros/humble/setup.bash

NAME="$(date +%Y%m%d_%H%M%S)"
[ -n "$LABEL" ] && NAME="${NAME}_${LABEL}"
mkdir -p "$BAG_DIR"

TOPICS=(
  '/wauv/.*'
  '/mavros/state'
  '/mavros/imu/data'
  '/mavros/imu/data_raw'
  '/mavros/imu/mag'
  '/mavros/global_position/rel_alt'
  '/mavros/local_position/odom'
  '/joystick'
  '/mavros/odometry/out'
)

# join with | for the regex
REGEX="^($(IFS='|'; echo "${TOPICS[*]}"))$"

exec ros2 bag record -e "$REGEX" -s "$STORAGE" -o "$BAG_DIR/$NAME"
