import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction,
                             SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg          = get_package_share_directory('second_project')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg, 'config', 'nav2.yaml')
    rviz_config = os.path.join(pkg, 'config', 'navigation.rviz')
    world_file  = os.path.join(pkg, 'maps', 'scout_world.world')
    map_yaml    = os.path.join(pkg, 'maps', 'my_map.yaml')
    csv_file    = os.path.join(pkg, 'csv', 'goals.csv')

    # Stage simulator.
    # Reads scout_world.world which loads my_map.pgm as the environment bitmap.
    # Publishes: /base_scan (BEST_EFFORT), /odom, /tf, /clock
    # Subscribes: /cmd_vel
    # TF tree it creates: odom -> base_footprint -> base_link -> base_laser_link
    stage = Node(
        package='stage_ros2_stageros',
        executable='stageros',
        name='stageros',
        arguments=[world_file],
        parameters=[{
            'use_sim_time': True,
            'base_watchdog_timeout': 0.2,
            # Publish odom TF one step behind to avoid timestamp collisions with LaserScan
            # 'delay_odom_tf_by_one_update': True,
        }],
        output='screen'
    )

    # Loads my_map.yaml and publishes /map.
    # yaml_filename overrides the empty string in nav2.yaml — avoids absolute paths in source.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[nav2_params, {'yaml_filename': map_yaml, 'use_sim_time': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen'
    )

    # AMCL particle filter localization.
    # Compares /base_scan against /map to estimate the robot's pose.
    # Publishes: map->odom transform, /particle_cloud, /amcl_pose
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[nav2_params, {
            'use_sim_time': True,
            'scan_topic': '/base_scan',
            'set_initial_pose': True,
            'always_reset_initial_pose': True,
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen'
    )

    # Lifecycle manager for map_server and amcl.
    # Nav2 nodes are lifecycle nodes: they do nothing until configured and activated.
    # This manager drives them through unconfigured->inactive->active automatically.
    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
        output='screen'
    )

    # Nav2 navigation stack: planner, controller, bt_navigator, behavior_server,
    # smoother, velocity_smoother, waypoint_follower — all from nav2_bringup.
    # We pass our nav2.yaml as params_file and it configures everything.
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'namespace':       '',
            'use_sim_time':    'true',
            'autostart':       'true',
            'params_file':     nav2_params,
            'use_composition': 'False',
            'use_respawn':     'False',
            'log_level':       'info',
        }.items(),
    )

    # Goal publisher: delayed 10s so Nav2 lifecycle activation completes before
    # the action client tries to connect. Log shows activation takes ~4s on this machine.
    goal_publisher = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='second_project',
                executable='goal_publisher_node',
                name='goal_publisher_node',
                parameters=[{
                    'csv_file':     csv_file,
                    'goal_timeout': 120.0,
                    'use_sim_time': True,
                }],
                output='screen'
            )
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        stage,
        map_server,
        amcl,
        lifecycle_localization,
        nav2_navigation,
        goal_publisher,
        rviz,
    ])
