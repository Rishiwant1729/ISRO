from setuptools import find_packages, setup

package_name = 'irocu_vio_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ov_to_vio = irocu_vio_bridge.ov_to_vio:main',
	    'phase2_health_check = irocu_vio_bridge.phase2_health_check:main',
            'phase2_takeoff_land = irocu_vio_bridge.phase2_takeoff_land:main',
	    'phase3_velocity_test = irocu_vio_bridge.phase3_velocity_test:main',
	    'phase3_square_path = irocu_vio_bridge.phase3_square_path:main',
	    'phase3_keyboard_control = irocu_vio_bridge.phase3_keyboard_control:main',
	    'phase3_square_yaw_hold = irocu_vio_bridge.phase3_square_yaw_hold:main',
	    'phase3_keyboard_rth = irocu_vio_bridge.phase3_keyboard_rth:main',
	    'phase3_lawnmower_path = irocu_vio_bridge.phase3_lawnmower_path:main',
            'phase3_lawnmower_aruco = irocu_vio_bridge.phase3_lawnmower_aruco:main',
            'phase3_keyboard_rth_aruco_offset = irocu_vio_bridge.phase3_keyboard_rth_aruco_offset:main',
            'phase3_aruco_detector = irocu_vio_bridge.phase3_aruco_detector:main',
            'phase3_aruco_detector_custom = irocu_vio_bridge.phase3_aruco_detector_custom:main',
	    'phase3_keyboard_rth_aruco = irocu_vio_bridge.phase3_keyboard_rth_aruco:main',
	    'image_capture_sender_node = irocu_vio_bridge.image_capture_sender_node:main',
	    'downward_camera_publisher = irocu_vio_bridge.downward_camera_publisher:main',
            'image_capture_sender_topic_node = irocu_vio_bridge.image_capture_sender_topic_node:main',
        'dashboard_agent = irocu_vio_bridge.dashboard_agent:main',
        'control_server = irocu_vio_bridge.control_server:main',
        ],
    },
)
