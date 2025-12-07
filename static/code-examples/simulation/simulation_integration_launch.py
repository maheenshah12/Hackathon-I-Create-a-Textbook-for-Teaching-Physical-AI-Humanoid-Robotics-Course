#!/usr/bin/env python3
"""
Launch file for Simulation Integration Example

This launch file demonstrates how to launch the complete simulation integration
system with Gazebo, Unity-style simulation, and ROS 2 coordination.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    run_gazebo = LaunchConfiguration('run_gazebo')
    run_unity_sim = LaunchConfiguration('run_unity_sim')
    run_integration_node = LaunchConfiguration('run_integration_node')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_run_gazebo = DeclareLaunchArgument(
        'run_gazebo',
        default_value='false',
        description='Whether to run Gazebo simulation'
    )

    declare_run_unity_sim = DeclareLaunchArgument(
        'run_unity_sim',
        default_value='false',
        description='Whether to run Unity-style simulation'
    )

    declare_run_integration_node = DeclareLaunchArgument(
        'run_integration_node',
        default_value='true',
        description='Whether to run the simulation integration node'
    )

    # Gazebo launch (if requested)
    # Note: In a real implementation, this would launch actual Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        condition=IfCondition(run_gazebo)
    )

    # Unity-style simulation node (simulated in our integration node)
    unity_sim_node = Node(
        package='simulation_integration',
        executable='unity_sim_node',  # This would be a separate node in real implementation
        name='unity_sim_node',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(run_unity_sim)
    )

    # Main simulation integration node
    sim_integration_node = Node(
        package='simulation_integration',
        executable='simulation_integration_node',  # This executable is our main integration script
        name='simulation_integration',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(run_integration_node),
        remappings=[
            ('/cmd_vel', '/input_cmd_vel'),
            ('/odom', '/output_odom'),
            ('/scan', '/output_scan'),
            ('/imu', '/output_imu'),
            ('/camera/image_raw', '/output_camera'),
        ]
    )

    # RViz2 for visualization
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('simulation_integration'),
        'rviz',
        'simulation_integration.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(run_integration_node)
    )

    # Robot state publisher for visualization
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(run_integration_node)
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_run_gazebo)
    ld.add_action(declare_run_unity_sim)
    ld.add_action(declare_run_integration_node)

    # Add actions to launch description
    ld.add_action(gazebo_launch)
    ld.add_action(unity_sim_node)
    ld.add_action(sim_integration_node)
    ld.add_action(rviz_node)
    ld.add_action(robot_state_publisher)

    return ld