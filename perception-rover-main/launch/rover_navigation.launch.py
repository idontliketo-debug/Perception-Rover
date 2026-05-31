import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            TimerAction, IncludeLaunchDescription)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('perception_rover')
    bringup_dir = get_package_share_directory('nav2_bringup')
    tb3_desc_dir = get_package_share_directory('turtlebot3_description')

    world_file = os.path.join(pkg_dir, 'worlds', 'perception_world.sdf')
    tb3_model = os.path.join(pkg_dir, 'models', 'tb3', 'tb3_waffle_rgb.sdf')
    bridge_cfg = os.path.join(pkg_dir, 'config', 'bridge.yaml')
    tb3_urdf = os.path.join(tb3_desc_dir, 'urdf', 'turtlebot3_waffle.urdf')

    headless = LaunchConfiguration('headless', default='false')

    # Kill old Gazebo
    gz_kill = ExecuteProcess(
        cmd=['bash', '-c',
             'pkill -9 -f "gz sim" 2>/dev/null; sleep 1; true'],
        output='screen',
    )

    # Start Gazebo with our arena world
    gz_sim = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-s', world_file],
            output='screen',
        )],
    )

    gz_gui = TimerAction(
        period=4.0,
        condition=UnlessCondition(headless),
        actions=[ExecuteProcess(
            cmd=['gz', 'sim', '-g'],
            output='screen',
        )],
    )

    # Bridge with our config (includes /scan, /image, /odom, /tf, /cmd_vel, etc)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_cfg}],
        output='screen',
    )

    # Robot State Publisher — publishes static TF from URDF (critical for
    # base_footprint → base_link → base_scan, etc. transforms)
    # The URDF uses xacro ${namespace} which robot_state_publisher doesn't
    # resolve automatically, so strip it.
    urdf_content = open(tb3_urdf).read().replace('${namespace}', '')
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': urdf_content,
            'use_sim_time': True,
            'frame_prefix': '',
        }],
    )

    # Spawn TB3 waffle into our arena
    spawn_tb3 = TimerAction(
        period=6.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                 '-world', 'perception_world',
                 '-file', tb3_model,
                 '-name', 'turtlebot3_waffle',
                 '-x', '0', '-y', '0', '-z', '0.15',
                 '-allow_renaming', 'false'],
            output='screen',
        )],
    )

    # Static map → odom transform (robot starts at origin in the map)
    # This replaces AMCL localization since we know the exact starting position
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
    )

    # Map server — loads the pre-built arena map
    arena_map_yaml = os.path.join(pkg_dir, 'maps', 'arena_map.yaml')
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': arena_map_yaml, 'use_sim_time': True}],
    )

    # Lifecycle manager for map server (activates it so map is published)
    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
            'use_sim_time': True,
        }],
    )

    # Nav2 bringup — pre-built map mode (no SLAM, no AMCL)
    arena_map = os.path.join(pkg_dir, 'maps', 'arena_map.yaml')
    nav2 = TimerAction(
        period=15.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'slam': 'False',
                'map': arena_map,
                'use_sim_time': 'True',
                'params_file': os.path.join(
                    bringup_dir, 'params', 'nav2_params.yaml'),
                'autostart': 'True',
                'use_composition': 'False',
                'use_respawn': 'True',
                'use_localization': 'False',
            }.items(),
        )],
    )

    # Perception nodes
    vision = Node(
        package='perception_rover',
        executable='vision_node',
        name='vision_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    lidar = Node(
        package='perception_rover',
        executable='lidar_node',
        name='lidar_node',
        respawn=True,
        respawn_delay=2.0,
    )

    mission = TimerAction(
        period=28.0,
        actions=[Node(
            package='perception_rover',
            executable='mission_orchestrator',
            name='mission_orchestrator',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='Skip Gazebo GUI'),
        gz_kill,
        gz_sim,
        gz_gui,
        bridge,
        robot_state_pub,
        map_odom_tf,
        map_server,
        map_server_lifecycle,
        spawn_tb3,
        nav2,
        vision,
        lidar,
        mission,
    ])
