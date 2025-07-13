#!/bin/bash

# Usage: ./load_world.sh "New World"

WORLD_NAME="$1"

if [ -z "$WORLD_NAME" ]; then
  echo "Usage: $0 <world name>"
  exit 1
fi

# Get the container ID of the first running container
CONTAINER_ID=$(sudo docker ps -q | head -n 1)

if [ -z "$CONTAINER_ID" ]; then
  echo "No running Docker containers found."
  exit 1
fi

# Define source and target paths
SOURCE_DIR="./${WORLD_NAME}"
TARGET_DIR="/ws/minecraft_ros2/run/saves/${WORLD_NAME}"

# Check if the source world directory exists
if [ ! -d "$SOURCE_DIR" ]; then
  echo "World '${WORLD_NAME}' not found in ./minecraft_worlds_backup/"
  exit 1
fi

# Copy world directory back into the container
sudo docker cp "$SOURCE_DIR" "$CONTAINER_ID":"$TARGET_DIR"

if [ $? -eq 0 ]; then
  echo "World '${WORLD_NAME}' successfully restored to container."
else
  echo "Failed to restore world '${WORLD_NAME}'."
  exit 1
fi
