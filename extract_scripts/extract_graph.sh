#!/bin/bash

### Lazy solution to syncing files to container's files. Run from root. ###

PLOT_NAME="$1"

if [ -z "$PLOT_NAME" ]; then
  echo "Usage: $0 <plot name>"
  exit 1
fi

OUTPUT_DIR="../figures"

# Create output directory if it doesn't exist
mkdir -p "$OUTPUT_DIR"


# Get the container ID of the first running container
CONTAINER_ID=$(sudo docker ps -q | head -n 1)

# Check if a container was found
if [ -z "$CONTAINER_ID" ]; then
  echo "No running Docker containers found."
  exit 1
fi

# Execute bash in the container

sudo docker cp "$CONTAINER_ID":"/ws/custom/custom_ws/${PLOT_NAME}" "$OUTPUT_DIR"

if [ $? -eq 0 ]; then
  echo "Plot '${PLOT_NAME}' successfully copied to ${OUTPUT_DIR}/"
else
  echo "Failed to copy plot '${POLICY_NAME}'"
  exit 1
fi

