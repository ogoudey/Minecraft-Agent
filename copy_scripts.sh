#!/bin/bash

### Lazy solution to syncing files to container's files. Run from root. ###

# Get the container ID of the first running container
CONTAINER_ID=$(sudo docker ps -q | head -n 1)

# Check if a container was found
if [ -z "$CONTAINER_ID" ]; then
  echo "No running Docker containers found."
  exit 1
fi

# Execute bash in the container
sudo docker exec "$CONTAINER_ID" mkdir -p /ws/custom

sudo docker cp src/custom_ws "$CONTAINER_ID":/ws/custom/

sudo docker cp src/quick_start.sh "$CONTAINER_ID":/ws/minecraft_ros2/

