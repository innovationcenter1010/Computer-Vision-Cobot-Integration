#!/usr/bin/env bash
# Source ROS + the workspace install tree, then exec the container command.
set -e

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

exec "$@"
