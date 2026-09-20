#!/bin/bash
set +u
source /opt/ros/humble/setup.bash
source /data/sf_code/ros_ws/install/setup.bash
export PYTHONUNBUFFERED=1
cd /data/sf_code/agent_ws/src/omni_node/scripts || exit 1
python3 probe_cloud_width.py
