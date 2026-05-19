from setuptools import find_packages, setup

package_name = 'palfinger_teleop'

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
    maintainer='local',
    maintainer_email='local@todo.todo',
    description='Crane teleoperation nodes (e.g., Xbox/joystick mapping).',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'camera_viewer = palfinger_teleop.camera_viewer:main',
            'hmi_range_monitor = palfinger_teleop.hmi_range_monitor:main',
            'hmi_range_viewer = palfinger_teleop.hmi_range_viewer:main',
            'snap_executor = palfinger_teleop.snap_executor:main',
            'snap_manager = palfinger_teleop.snap_manager:main',
            'snap_target_provider = palfinger_teleop.snap_target_provider:main',
            'teleop_joy = palfinger_teleop.teleop_joy:main',
            'teleop_operator_training = palfinger_teleop.teleop_operator_training:main',
        ],
    },
)
