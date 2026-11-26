#!/usr/bin/env bash
# Source after env.sh. Process-local configuration for this detection workspace.
# RMW's legacy localhost-only override otherwise replaces our SHM descriptor.
unset ROS_LOCALHOST_ONLY FASTDDS_BUILTIN_TRANSPORTS
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
export FASTRTPS_DEFAULT_PROFILES_FILE="$VISUAL_PERCEPTION_ROOT/config/fastdds_local_large_cloud.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
