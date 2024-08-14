#!/bin/bash
ros2 launch usb_cam camera.launch.py
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.02 --ros-args -r image:=/image_raw -p camera:=/camera1