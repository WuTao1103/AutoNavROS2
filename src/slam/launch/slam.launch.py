from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch import conditions
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('slam')

    # Parameters file
    params_file = LaunchConfiguration('params_file')
    use_simple_odom = LaunchConfiguration('use_simple_odom')

    # Declare launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'slam_params.yaml'),
        description='Path to the SLAM parameters file')

    declare_use_simple_odom_cmd = DeclareLaunchArgument(
        'use_simple_odom',
        default_value='false',
        description='Use simple odometry node (set to true if no Gazebo odom available)')

    # Static transform publishers
    # Map to odom transform (identity for simple SLAM)
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # Base link to laser transform (TurtleBot3 configuration)
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0.064', '0', '0.122', '0', '0', '0', 'base_link', 'base_scan'],
        output='screen'
    )

    # Simple odometry node (optional - use when Gazebo odom not available)
    simple_odom_node = Node(
        package='slam',
        executable='simple_odometry.py',
        name='simple_odometry',
        parameters=[params_file],
        output='screen',
        condition=conditions.IfCondition(use_simple_odom)
    )

    # Occupancy grid mapper
    occupancy_mapper_node = Node(
        package='slam',
        executable='occupancy_grid_mapper.py',
        name='occupancy_grid_mapper',
        parameters=[params_file],
        output='screen',
        remappings=[
            ('/scan', '/scan'),
            ('/odom', '/odom'),
            ('/map', '/map')
        ]
    )

    return LaunchDescription([
        declare_params_file_cmd,
        declare_use_simple_odom_cmd,
        map_to_odom_tf,
        base_to_laser_tf,
        simple_odom_node,
        occupancy_mapper_node
    ])