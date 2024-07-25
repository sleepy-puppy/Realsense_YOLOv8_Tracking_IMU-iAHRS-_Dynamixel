# Realsense D455
# YOLOv8 Deepsort Object Detection
# Tracking selected object with Dynamixel motors
# Gimbal with IMU device(iAHRS)
# PyQt5 application
# ROS2 communication

# How to run
First, turn on ROS2
- cd your_ros_ws
- source install/setup.bash
- cd src/imu_realsense_tracking_pyqt (where python files are.)

Run the files with this order
[realsense_pyqt_deepsort.py -> iAHRS_ros.py -> dynamixel_position_imu_ros.py]
