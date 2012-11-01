#!/bin/bash
rosbag record -b 0 -o $1 /usb_cam/camera_info /usb_cam/image_raw /handle/sensors/raw /RightArm/WAM/arm_positions
