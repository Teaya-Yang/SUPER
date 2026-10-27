#!/usr/bin/env bash
# Record the topics extract_replans.py needs. Run inside the ROS container
# while the planner is running, e.g.:
#   bash record_replans.sh ~/ws_super/bags/dense.bag 40
set -euo pipefail
out=${1:?usage: record_replans.sh OUT.bag [SECONDS]}
secs=${2:-40}
timeout "$secs" rosbag record -O "$out" \
    /fsm_node/visualization/frontend_path \
    /fsm_node/visualization/exp_traj \
    /fsm_node/visualization/exp_sfc \
    /fsm_node/visualization/committed_traj \
    /lidar_slam/odom /global_pc || true
