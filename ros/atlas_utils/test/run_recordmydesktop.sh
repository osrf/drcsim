#!/bin/bash
set -ex

# Wait for all the other nodes to hook up
sleep 10

LOG_DIR="${HOME}/.ros"
LOG_FILE="${LOG_DIR}/gazebo.ogv"
mkdir -p ${LOG_DIR}

# Start video recording
recordmydesktop --no-sound --windowid=$(xdotool search --name gazebo | tail -1) ${LOG_FILE}
