from setuptools import find_packages, setup

package_name = 'laundry_bot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/teleop_record.launch.py',
          'launch/bringup_virtual.launch.py']),
        ('share/' + package_name + '/config',
         ['config/arm.yaml', 'config/ur7e_like.yaml',
          'config/teleop.yaml', 'config/record.yaml',
          'config/fake_waypoints.yaml']),
        ('share/' + package_name + '/docs',
         ['../docs/quest2ros2_setup.md']),
    ],
    install_requires=['setuptools', 'numpy', 'pyyaml'],
    zip_safe=True,
    maintainer='Paden',
    maintainer_email='paden@example.com',
    description='Laundry bot ROS 2 nodes: arm driver, VR teleop, LeRobot recorder',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_driver_node = laundry_bot.arm_driver_node:main',
            'teleop_quest_node = laundry_bot.teleop_quest_node:main',
            'fake_teleop_node = laundry_bot.fake_teleop_node:main',
            'fake_camera_node = laundry_bot.fake_camera_node:main',
            'lerobot_recorder_node = laundry_bot.lerobot_recorder_node:main',
            'record_cli = laundry_bot.record_cli:main',
            'estop = laundry_bot.estop:main',
        ],
    },
)
