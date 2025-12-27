#!/usr/bin/env python3
"""
Launch Gazebo with apriltag_slam robot (camera + IMU)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('apriltag_slam')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    
    # Paths
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    # Change this line:
    world_file = os.path.join(pkg_share, 'worlds', 'apriltag_world.world')

    
    # Process xacro to URDF
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Gazebo launch file (official ROS2 way)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file if os.path.exists(world_file) else '',
            'verbose': 'true'
        }.items()
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', 'apriltag_bot',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.5'
        ]
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
