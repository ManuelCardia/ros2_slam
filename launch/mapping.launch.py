import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('second_project')

    slam_params  = os.path.join(pkg, 'config', 'slam_toolbox.yaml')
    pc2ls_params = os.path.join(pkg, 'config', 'pointcloud_to_laserscan.yaml')
    rviz_config  = os.path.join(pkg, 'config', 'mapping.rviz')

    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='',
        description='Absolute path to the ROS2 bag directory'
    )

    # Replay the bag at real speed.
    # --clock publishes /clock so nodes with use_sim_time=true sync to bag time.
    # This also replays /tf_static which gives SLAM the sensor mounting position.
    bag_player = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag_file'),
            '--clock',
            '--rate', '1.0',
        ],
        output='screen',
        name='bag_player'
    )

    # Converts 3D PointCloud2 to 2D LaserScan.
    # Input  topic remapped: cloud_in  -> /ugv/rslidar_points
    # Output topic remapped: scan      -> /base_scan
    # /base_scan is what SLAM Toolbox expects (scan_topic in slam_toolbox.yaml)
    # and also what Stage publishes in Task 2, so the name is consistent.
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        parameters=[pc2ls_params, {'use_sim_time': True}],
        remappings=[
            ('cloud_in', '/ugv/rslidar_points'),
            ('scan',     '/base_scan'),
        ],
        output='screen'
    )

    # SLAM Toolbox reads /base_scan + /tf + /tf_static and builds /map.
    # Frames configured in slam_toolbox.yaml match the bag: UGV_odom, UGV_base_link.
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params, {'use_sim_time': True}],
        output='screen'
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
        bag_file_arg,
        bag_player,
        pointcloud_to_laserscan,
        slam_toolbox,
        rviz,
    ])
