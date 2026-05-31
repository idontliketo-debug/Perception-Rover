from setuptools import find_packages, setup

package_name = 'perception_rover'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/rover_navigation.launch.py']),
        ('share/' + package_name + '/config',
            ['config/nav2_params.yaml', 'config/bridge.yaml']),
        ('share/' + package_name + '/worlds',
            ['worlds/perception_world.sdf']),
        ('share/' + package_name + '/models',
            ['models/yolov5n.onnx']),
        ('share/' + package_name + '/models/tb3',
            ['models/tb3/tb3_waffle_rgb.sdf']),
        ('share/' + package_name + '/maps',
            ['maps/arena_map.yaml', 'maps/arena_map.pgm']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Perception rover — YOLO + LiDAR + Nav2 on Turtlebot3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = perception_rover.vision_node:main',
            'lidar_node = perception_rover.lidar_node:main',
            'mission_orchestrator = perception_rover.mission_orchestrator:main',
        ],
    },
)
