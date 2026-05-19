from setuptools import find_packages, setup

package_name = 'palfinger_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='local',
    maintainer_email='local@todo.todo',
    description='Crane control nodes converting CraneCommand to joint velocity commands.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crane_controller = palfinger_control.crane_controller:main',
            'ship_dp_hold = palfinger_control.ship_dp_hold:main',
        ],
    },
)
