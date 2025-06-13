#!/bin/bash

### Used for hopping into the container ###

# Get the container ID of the first running container
CONTAINER_ID=$(sudo docker ps -q | head -n 1)

# Check if a container was found
if [ -z "$CONTAINER_ID" ]; then
  echo "No running Docker containers found."
  exit 1
fi

# Execute bash in the container
sudo docker exec -it "$CONTAINER_ID" bash
