from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import conditions
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directories
    slam_pkg_dir = get_package_share_directory('slam')
    path_planning_pkg_dir = get_package_share_directory('path_planning')

    # Parameters files
    slam_params_file = LaunchConfiguration('slam_params_file')
    planning_params_file = LaunchConfiguration('planning_params_file')
    use_simple_odom = LaunchConfiguration('use_simple_odom')
    use_gazebo = LaunchConfiguration('use_gazebo')

    # Declare launch arguments
    declare_slam_params_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(slam_pkg_dir, 'config', 'slam_params.yaml'),
        description='Path to the SLAM parameters file')

    declare_planning_params_cmd = DeclareLaunchArgument(
        'planning_params_file',
        default_value=os.path.join(path_planning_pkg_dir, 'config', 'path_planning_params.yaml'),
        description='Path to the path planning parameters file')

    declare_use_simple_odom_cmd = DeclareLaunchArgument(
        'use_simple_odom',
        default_value='false',
        description='Use simple odometry node (set to true if no Gazebo odom)')

    declare_use_gazebo_cmd = DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',
        description='Use Gazebo simulation (set to true for real robot)')

    # Static transform publishers
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0.064', '0', '0.122', '0', '0', '0', 'base_link', 'base_scan'],
        output='screen'
    )

    # Simple odometry node (when not using Gazebo)
    simple_odom_node = Node(
        package='slam',
        executable='simple_odometry.py',
        name='simple_odometry',
        parameters=[slam_params_file],
        output='screen',
        condition=conditions.IfCondition(use_simple_odom)
    )

    # SLAM occupancy grid mapper
    occupancy_mapper_node = Node(
        package='slam',
        executable='occupancy_grid_mapper.py',
        name='occupancy_grid_mapper',
        parameters=[slam_params_file],
        output='screen'
    )

    # Path planning service (start after SLAM is ready)
    path_planning_service_node = TimerAction(
        period=3.0,  # Wait for SLAM to initialize
        actions=[
            Node(
                package='path_planning',
                executable='path_planning_service.py',
                name='path_planning_service',
                parameters=[planning_params_file],
                output='screen'
            )
        ]
    )

    # Goal pose bridge (connects RViz 2D Nav Goal to path planning)
    goal_pose_bridge_node = TimerAction(
        period=3.5,
        actions=[
            Node(
                package='path_planning',
                executable='goal_pose_bridge.py',
                name='goal_pose_bridge',
                output='screen'
            )
        ]
    )

    # Pure pursuit controller
    pure_pursuit_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='path_planning',
                executable='pure_pursuit_controller.py',
                name='pure_pursuit_controller',
                parameters=[planning_params_file],
                output='screen'
            )
        ]
    )

    # Optional: include map simulator for testing (when not using Gazebo)
    map_simulator_node = Node(
        package='path_planning',
        executable='simple_map_simulator.py',
        name='simple_map_simulator',
        parameters=[{
            'map_type': 'room_with_obstacles',
            'map_width': 100,
            'map_height': 100,
            'resolution': 0.1,
            'origin_x': -5.0,
            'origin_y': -5.0,
            'publish_rate': 1.0
        }],
        output='screen',
        condition=conditions.UnlessCondition(use_gazebo)
    )

    return LaunchDescription([
        declare_slam_params_cmd,
        declare_planning_params_cmd,
        declare_use_simple_odom_cmd,
        declare_use_gazebo_cmd,
        map_to_odom_tf,
        base_to_laser_tf,
        simple_odom_node,
        occupancy_mapper_node,
        path_planning_service_node,
        goal_pose_bridge_node,
        pure_pursuit_node,
        # Uncomment the line below for testing without Gazebo
        # map_simulator_node
    ])