#!/bin/bash


# Usage: ./save_world.sh "New World"

WORLD_NAME="$1"

if [ -z "$WORLD_NAME" ]; then
  echo "Usage: $0 <world name>"
  exit 1
fi

# Define output directory
OUTPUT_DIR="../minecraft_world_saves"

# Create output directory if it doesn't exist
mkdir -p "$OUTPUT_DIR"

CONTAINER_ID=$(sudo docker ps -q | head -n 1)

# Copy the world from inside the container
sudo docker cp "$CONTAINER_ID":"/ws/minecraft_ros2/run/saves/${WORLD_NAME}" "$OUTPUT_DIR"

if [ $? -eq 0 ]; then
  echo "World '${WORLD_NAME}' successfully copied to ${OUTPUT_DIR}/"
else
  echo "Failed to copy world '${WORLD_NAME}'"
  exit 1
fi
