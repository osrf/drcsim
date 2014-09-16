#!/bin/bash

# Wait for all the other nodes to hook up
sleep 10

# Log data relevant to performance testing
set -ex

rosbag record -O controller_statistics /atlas/controller_statistics
