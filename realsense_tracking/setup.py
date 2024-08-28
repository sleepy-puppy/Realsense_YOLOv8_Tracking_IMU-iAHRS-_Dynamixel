from setuptools import setup

package_name = 'realsense_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'realsense_tracking.realsense_pose_1',
        'realsense_tracking.iAHRS_2',
        'realsense_tracking.dynamixel_3',
        'realsense_tracking.realtime_audio_pc_4',
        'realsense_tracking.witmotion_imu_ros_5'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moon',
    maintainer_email='moon@todo.todo',
    description='ROS 2 package for Gimbal tracking system',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realsense_pose_node = realsense_tracking.realsense_pose_1:main',
            'iAHRS_node = realsense_tracking.iAHRS_2:main',
            'dynamixel_node = realsense_tracking.dynamixel_3:main',
            'realtime_audio_pc_node = realsense_tracking.realtime_audio_pc_4:main',
            'witmotion_imu_node = realsense_tracking.witmotion_imu_ros_5:main',
        ],
    },
)
