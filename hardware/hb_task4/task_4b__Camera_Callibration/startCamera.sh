#!/bin/bash
ros2 launch usb_cam camera.launch.py &
ros2 launch image_proc image_proc.launch.py &
python /home/tanishq/eyrc_hb/hb_task_4_ws/task_4b__Camera_Callibration/feedback.py
# python /home/tanishq/eyrc_hb/hb_task_4_ws/task_4b__Camera_Callibration/feedback.py debug
# python /home/tanishq/eyrc_hb/hb_task_4_ws/task_4b__Camera_Callibration/feedback.py print_loc
# python /home/tanishq/eyrc_hb/hb_task_4_ws/task_4b__Camera_Callibration/feedback.py debug print_loc