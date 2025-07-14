#!/bin/bash

POLICY_NAME="$1"

if [ -z "$POLICY_NAME" ]; then
  echo "Usage: $0 <policy name>"
  exit 1
fi

# Define output directory
OUTPUT_DIR="../policy_saves"

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

sudo docker cp "$CONTAINER_ID":/ws/custom/checkpoints/" .
# Copy the policy from inside the container
sudo docker cp "$CONTAINER_ID":"/ws/custom/checkpoints/${POLICY_NAME}" "$OUTPUT_DIR"

if [ $? -eq 0 ]; then
  echo "Policy '${POLICY_NAME}' successfully copied to ${OUTPUT_DIR}/"
else
  echo "Failed to copy policy '${POLICY_NAME}'"
  exit 1
fi
