from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Madgwick IMU filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
            }],
            remappings=[
                ('imu/data_raw', '/camera/d455/imu'),
                ('imu/data', '/camera/d455/imu_filtered'),
            ],
        ),

        # RTAB-Map RGB-D odometry
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'd455_link',
                'publish_tf': False,
                'approx_sync': True,
                'queue_size': 50,
                'wait_imu_to_init': True,
                'always_check_imu_tf': False,

                'Odom/Strategy': '0',
                'Odom/ResetCountdown': '1',
                'Odom/GuessMotion': 'true',
                'OdomF2M/MaxSize': '1000',

                'Vis/MinInliers': '25',
                'Vis/InlierDistance': '0.05',
                'Vis/MaxFeatures': '1000',

                'RGBD/OptimizeMaxError': '2.0',
            }],
            remappings=[
                ('rgb/image', '/camera/d455/color/image_raw'),
                ('depth/image', '/camera/d455/aligned_depth_to_color/image_raw'),
                ('rgb/camera_info', '/camera/d455/color/camera_info'),
                ('imu', '/camera/d455/imu_filtered'),
                ('odom', '/vio/odometry'),
            ],
        ),

        # VIO → MAVROS
        Node(
            package='topic_tools',
            executable='relay',
            name='vio_to_mavros_relay',
            output='screen',
            arguments=[
                '/vio/odometry',
                '/mavros/odometry/out',
            ],
        ),
    ])
