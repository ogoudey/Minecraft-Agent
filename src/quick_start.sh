#!/bin/bash

### Source all three ROS workspaces

cd ../custom/custom_ws
source /opt/ros/humble/setup.bash
source ../../ros2_java_ws/install/setup.bash
colcon build
source install/setup.bash

### Source python environment

source ../.venv/bin/activate
