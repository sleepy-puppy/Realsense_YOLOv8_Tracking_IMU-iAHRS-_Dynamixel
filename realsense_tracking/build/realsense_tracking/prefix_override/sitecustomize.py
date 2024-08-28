import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/moon/ros2_ws/src/realsense_tracking/install/realsense_tracking'
