#!/bin/bash

# Run from /ws/minecraft_ros2/
cd ../custom
if [ -d ".venv" ]; then
    echo "Python venv found. Sourcing..."
    source .venv/bin/activate
else
    echo "Python venv not found. Creating one..."

    apt update
    apt install -y python3.10-venv

    
    python3 -m venv .venv
    echo "Python venv created. Sourcing..."
    source .venv/bin/activate
    
    pip install --upgrade pip
    pip install pygame numpy torch pyyaml cv_bridge opencv-python matplotlib
fi



cd custom_ws
echo "Sourcing ROS underlay..."
source /opt/ros/humble/setup.bash
echo "Sourcing Java-ROS overlay..."
source ../../ros2_java_ws/install/setup.bash
colcon build
echo "Sourcing custom overlay..."
source install/setup.bash
