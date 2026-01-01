#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('apriltag_slam')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_share, 'urdf', 'robot_effort.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'model_world_new.world')
    rviz_config = os.path.join(pkg_share, 'config', 'view_slam.rviz')
    ctrl_yaml = os.path.join(pkg_share, 'config', 'effort.yaml')

    robot_description_config = xacro.process_file(xacro_file, mappings={'config_file': ctrl_yaml})
    robot_description = {'robot_description': robot_description_config.toxml()}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file, 'extra_gazebo_args': '--ros-args --log-level error'}.items()
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot', '-x', '0', '-y', '0', '-z', '0.1','-timeout','90'],
        output='screen' 
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    load_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    load_steering_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steering_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    load_imu_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        output='screen'
    )
    state_estimator = Node(
        package='apriltag_slam',
        executable='estimator',
        name='state_estimator',
        output='screen',
        parameters=[{'tag_size': 0.15}]
    )

    random_tag_mover = Node(
        package='apriltag_slam',
        executable='random_tag_mover',
        name='random_tag_mover',
        output='screen'
    )

    rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config],
    parameters=[{'use_sim_time': True}], # Add this
    output='screen'
    )

    ground_truth_tf_publisher = Node(
        package='apriltag_slam',
        executable='ground_truth_broadcaster',
        name='ground_truth_tf_publisher',
        parameters=[{'use_sim_time': True}], # Add this
        output='screen'
    )

    
   
    load_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_robot_state_publisher,
            on_exit=[
                load_joint_state_broadcaster,
                load_drive_controller,
                load_steering_controller,
                load_imu_broadcaster,
                rviz_node,
                state_estimator,
                # random_tag_mover,
            ]
        )
    )

    return LaunchDescription([
        #static_tf,
        ground_truth_tf_publisher,
        gazebo,
        node_robot_state_publisher,
        # spawn_entity,
        load_joint_state_broadcaster,
        load_drive_controller,
        load_steering_controller,
        load_imu_broadcaster,
        rviz_node,
        #load_controllers_after_spawn,
        
    ])
