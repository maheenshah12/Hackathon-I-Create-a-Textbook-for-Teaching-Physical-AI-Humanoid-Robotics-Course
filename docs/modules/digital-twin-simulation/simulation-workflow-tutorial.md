---
sidebar_position: 4
---

# Simulation Workflow Tutorial: Complete Digital Twin System

## Overview

This tutorial guides you through setting up and running a complete digital twin simulation system that integrates Gazebo, Unity-style simulation, and ROS 2. You'll learn how to create a comprehensive simulation environment that can be used for robotics education and development.

## Prerequisites

Before starting this tutorial, ensure you have:

1. **ROS 2 Humble** installed and properly configured
2. **Ubuntu 22.04 LTS** with Python 3.8+
3. **Gazebo Garden** installed (for Gazebo integration)
4. **Basic knowledge** of ROS 2 concepts
5. **Python libraries**: numpy, scipy, filterpy (for simulation examples)

## Installation and Setup

### 1. Install Required Dependencies

First, install the ROS 2 packages needed for simulation:

```bash
# Install Gazebo packages
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# Install visualization packages
sudo apt install ros-humble-rviz2 ros-humble-robot-state-publisher

# Install Python dependencies for simulation examples
pip install numpy scipy filterpy matplotlib
```

### 2. Create a ROS 2 Workspace

Create a new ROS 2 workspace for the simulation integration:

```bash
mkdir -p ~/simulation_ws/src
cd ~/simulation_ws
colcon build
source install/setup.bash
```

### 3. Create the Simulation Integration Package

Create a new ROS 2 package for the simulation integration:

```bash
cd ~/simulation_ws/src
ros2 pkg create --build-type ament_python simulation_integration
```

## Implementation Steps

### Step 1: Create the Main Simulation Integration Node

Create the main simulation integration file at `~/simulation_ws/src/simulation_integration/simulation_integration/simulation_integration_node.py`:

```python
#!/usr/bin/env python3
"""
Complete Simulation Integration Example

This script demonstrates integration between Gazebo, Unity (simulated), and ROS 2
for creating comprehensive digital twin simulations. It includes:
1. Gazebo simulation control
2. Unity-style sensor simulation
3. ROS 2 message handling
4. Multi-simulation coordination
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import threading
import time
from collections import deque
import random

class SimulationIntegrationNode(Node):
    def __init__(self):
        super().__init__('simulation_integration')

        # Simulation state
        self.robot_pose = Pose()
        self.robot_twist = Twist()
        self.simulation_time = 0.0
        self.real_time_factor = 1.0
        self.is_paused = False

        # Sensor simulation parameters
        self.laser_range = 10.0  # meters
        self.laser_angle_min = -math.pi / 2
        self.laser_angle_max = math.pi / 2
        self.laser_angle_increment = math.pi / 180  # 1 degree
        self.camera_resolution = (640, 480)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        # Transform broadcaster for TF tree
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Simulation control
        self.sim_control_sub = self.create_subscription(
            String, '/sim_control', self.sim_control_callback, 10)

        # Timer for simulation update (100 Hz)
        self.sim_timer = self.create_timer(0.01, self.simulation_step)

        # History for visualization
        self.pose_history = deque(maxlen=1000)

        # Environment objects for sensor simulation
        self.environment_objects = [
            {'type': 'wall', 'position': (3.0, 0.0), 'size': (0.2, 4.0)},
            {'type': 'wall', 'position': (-3.0, 0.0), 'size': (0.2, 4.0)},
            {'type': 'wall', 'position': (0.0, 3.0), 'size': (6.0, 0.2)},
            {'type': 'wall', 'position': (0.0, -3.0), 'size': (6.0, 0.2)},
            {'type': 'obstacle', 'position': (1.0, 1.0), 'size': (0.5, 0.5)},
            {'type': 'obstacle', 'position': (-1.5, -1.5), 'size': (0.8, 0.8)},
        ]

        self.get_logger().info('Simulation integration node initialized')

    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        self.robot_twist = msg

    def sim_control_callback(self, msg):
        """Handle simulation control commands"""
        command = msg.data.lower()

        if command == 'pause':
            self.is_paused = True
            self.get_logger().info('Simulation paused')
        elif command == 'resume':
            self.is_paused = False
            self.get_logger().info('Simulation resumed')
        elif command == 'reset':
            self.reset_simulation()
            self.get_logger().info('Simulation reset')
        elif command.startswith('set_rtf_'):
            try:
                rtf = float(command.split('_')[2])
                self.real_time_factor = max(0.1, min(10.0, rtf))
                self.get_logger().info(f'Real time factor set to {self.real_time_factor}')
            except ValueError:
                self.get_logger().error(f'Invalid RTF value in command: {command}')

    def reset_simulation(self):
        """Reset simulation to initial state"""
        self.robot_pose.position.x = 0.0
        self.robot_pose.position.y = 0.0
        self.robot_pose.position.z = 0.0
        self.robot_pose.orientation.x = 0.0
        self.robot_pose.orientation.y = 0.0
        self.robot_pose.orientation.z = 0.0
        self.robot_pose.orientation.w = 1.0
        self.robot_twist.linear.x = 0.0
        self.robot_twist.angular.z = 0.0
        self.simulation_time = 0.0
        self.pose_history.clear()

    def simulation_step(self):
        """Main simulation step"""
        if self.is_paused:
            return

        dt = 0.01  # 100 Hz

        # Update simulation time
        self.simulation_time += dt * self.real_time_factor

        # Update robot pose based on velocity commands (differential drive kinematics)
        self.update_robot_pose(dt)

        # Publish sensor data
        self.publish_odometry()
        self.publish_laser_scan()
        self.publish_imu_data()
        self.publish_camera_image()
        self.publish_visualization_markers()

        # Broadcast transforms
        self.broadcast_transforms()

    def update_robot_pose(self, dt):
        """Update robot pose using differential drive kinematics"""
        # Simple kinematic model
        linear_vel = self.robot_twist.linear.x
        angular_vel = self.robot_twist.angular.z

        # Get current orientation
        quat = self.robot_pose.orientation
        current_yaw = math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        )

        # Update position
        self.robot_pose.position.x += linear_vel * math.cos(current_yaw) * dt
        self.robot_pose.position.y += linear_vel * math.sin(current_yaw) * dt

        # Update orientation
        new_yaw = current_yaw + angular_vel * dt
        new_quat = R.from_euler('z', new_yaw).as_quat()
        self.robot_pose.orientation.x = new_quat[0]
        self.robot_pose.orientation.y = new_quat[1]
        self.robot_pose.orientation.z = new_quat[2]
        self.robot_pose.orientation.w = new_quat[3]

        # Keep robot within bounds (simple collision avoidance)
        self.robot_pose.position.x = max(-4.8, min(4.8, self.robot_pose.position.x))
        self.robot_pose.position.y = max(-4.8, min(4.8, self.robot_pose.position.y))

        # Store pose in history for visualization
        self.pose_history.append((self.robot_pose.position.x, self.robot_pose.position.y))

    def publish_odometry(self):
        """Publish odometry data"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose = self.robot_pose
        odom_msg.twist.twist = self.robot_twist

        # Set covariance (simplified)
        for i in range(36):
            if i % 7 == 0:  # Diagonal elements
                odom_msg.pose.covariance[i] = 0.1
                odom_msg.twist.covariance[i] = 0.1

        self.odom_pub.publish(odom_msg)

    def publish_laser_scan(self):
        """Publish simulated laser scan data"""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'

        scan_msg.angle_min = self.laser_angle_min
        scan_msg.angle_max = self.laser_angle_max
        scan_msg.angle_increment = self.laser_angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = self.laser_range

        # Calculate number of ranges
        num_ranges = int((self.laser_angle_max - self.laser_angle_min) / self.laser_angle_increment) + 1
        ranges = []

        # Simulate laser ranges by checking for obstacles in the environment
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        robot_yaw = math.atan2(
            2.0 * (self.robot_pose.orientation.w * self.robot_pose.orientation.z +
                   self.robot_pose.orientation.x * self.robot_pose.orientation.y),
            1.0 - 2.0 * (self.robot_pose.orientation.y * self.robot_pose.orientation.y +
                         self.robot_pose.orientation.z * self.robot_pose.orientation.z)
        )

        for i in range(num_ranges):
            angle = self.laser_angle_min + i * self.laser_angle_increment + robot_yaw
            range_val = self.laser_range  # Default to max range

            # Check for intersections with environment objects
            for obj in self.environment_objects:
                obj_x, obj_y = obj['position']
                size_x, size_y = obj['size']

                # Calculate intersection with this object
                end_x = robot_x + self.laser_range * math.cos(angle)
                end_y = robot_y + self.laser_range * math.sin(angle)

                # Simple rectangle intersection (axis-aligned bounding box)
                if self.check_line_box_intersection(robot_x, robot_y, end_x, end_y,
                                                  obj_x - size_x/2, obj_y - size_y/2,
                                                  obj_x + size_x/2, obj_y + size_y/2):
                    # Find the intersection point
                    intersection_dist = self.find_line_box_intersection_distance(
                        robot_x, robot_y, angle, obj_x, obj_y, size_x/2, size_y/2)
                    if intersection_dist < range_val:
                        range_val = intersection_dist

            # Add some noise to the range measurement
            range_val += random.gauss(0, 0.01)  # 1cm standard deviation
            range_val = max(0.1, min(self.laser_range, range_val))  # Clamp to valid range

            ranges.append(range_val)

        scan_msg.ranges = ranges
        scan_msg.intensities = [100.0] * len(ranges)  # Constant intensity

        self.scan_pub.publish(scan_msg)

    def check_line_box_intersection(self, x1, y1, x2, y2, min_x, min_y, max_x, max_y):
        """Check if line segment intersects with axis-aligned box"""
        # Check if the line intersects with any of the box edges
        # This is a simplified implementation

        # Check intersection with box boundaries
        # Left edge
        if self.line_intersects_vertical(x1, y1, x2, y2, min_x, min_y, max_y):
            return True
        # Right edge
        if self.line_intersects_vertical(x1, y1, x2, y2, max_x, min_y, max_y):
            return True
        # Bottom edge
        if self.line_intersects_horizontal(x1, y1, x2, y2, min_y, min_x, max_x):
            return True
        # Top edge
        if self.line_intersects_horizontal(x1, y1, x2, y2, max_y, min_x, max_x):
            return True

        return False

    def line_intersects_vertical(self, x1, y1, x2, y2, x_edge, y_min, y_max):
        """Check if line intersects with vertical edge"""
        if x1 == x2:  # Vertical line
            return x1 == x_edge and min(y1, y2) <= y_max and max(y1, y2) >= y_min

        # Calculate intersection point
        t = (x_edge - x1) / (x2 - x1)
        if 0 <= t <= 1:
            y_int = y1 + t * (y2 - y1)
            return y_min <= y_int <= y_max

        return False

    def line_intersects_horizontal(self, x1, y1, x2, y2, y_edge, x_min, x_max):
        """Check if line intersects with horizontal edge"""
        if y1 == y2:  # Horizontal line
            return y1 == y_edge and min(x1, x2) <= x_max and max(x1, x2) >= x_min

        # Calculate intersection point
        t = (y_edge - y1) / (y2 - y1)
        if 0 <= t <= 1:
            x_int = x1 + t * (x2 - x1)
            return x_min <= x_int <= x_max

        return False

    def find_line_box_intersection_distance(self, robot_x, robot_y, angle, obj_x, obj_y, half_size_x, half_size_y):
        """Find the distance to the nearest intersection with a box"""
        # Calculate the intersection points with the box edges
        # This is a simplified implementation
        end_x = robot_x + self.laser_range * math.cos(angle)
        end_y = robot_y + self.laser_range * math.sin(angle)

        # Box boundaries
        left = obj_x - half_size_x
        right = obj_x + half_size_x
        bottom = obj_y - half_size_y
        top = obj_y + half_size_y

        # Calculate intersection with each edge
        intersections = []

        # Left edge (vertical)
        if robot_x != end_x:  # Not vertical ray
            t = (left - robot_x) / (end_x - robot_x)
            if 0 <= t <= 1:
                y_int = robot_y + t * (end_y - robot_y)
                if bottom <= y_int <= top:
                    dist = math.sqrt((left - robot_x)**2 + (y_int - robot_y)**2)
                    intersections.append(dist)

        # Right edge (vertical)
        if robot_x != end_x:  # Not vertical ray
            t = (right - robot_x) / (end_x - robot_x)
            if 0 <= t <= 1:
                y_int = robot_y + t * (end_y - robot_y)
                if bottom <= y_int <= top:
                    dist = math.sqrt((right - robot_x)**2 + (y_int - robot_y)**2)
                    intersections.append(dist)

        # Bottom edge (horizontal)
        if robot_y != end_y:  # Not horizontal ray
            t = (bottom - robot_y) / (end_y - robot_y)
            if 0 <= t <= 1:
                x_int = robot_x + t * (end_x - robot_x)
                if left <= x_int <= right:
                    dist = math.sqrt((x_int - robot_x)**2 + (bottom - robot_y)**2)
                    intersections.append(dist)

        # Top edge (horizontal)
        if robot_y != end_y:  # Not horizontal ray
            t = (top - robot_y) / (end_y - robot_y)
            if 0 <= t <= 1:
                x_int = robot_x + t * (end_x - robot_x)
                if left <= x_int <= right:
                    dist = math.sqrt((x_int - robot_x)**2 + (top - robot_y)**2)
                    intersections.append(dist)

        return min(intersections) if intersections else self.laser_range

    def publish_imu_data(self):
        """Publish simulated IMU data"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Set orientation (same as robot pose)
        imu_msg.orientation = self.robot_pose.orientation

        # Set angular velocity (from twist command)
        imu_msg.angular_velocity.z = self.robot_twist.angular.z
        # Add some noise
        imu_msg.angular_velocity.x = random.gauss(0, 0.01)
        imu_msg.angular_velocity.y = random.gauss(0, 0.01)

        # Set linear acceleration (simplified)
        imu_msg.linear_acceleration.x = self.robot_twist.linear.x * 2.0  # Assume 2 m/s^2 per m/s velocity
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81  # Gravity

        self.imu_pub.publish(imu_msg)

    def publish_camera_image(self):
        """Publish simulated camera image"""
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_frame'
        img_msg.height = self.camera_resolution[1]
        img_msg.width = self.camera_resolution[0]
        img_msg.encoding = 'rgb8'
        img_msg.is_bigendian = 0
        img_msg.step = self.camera_resolution[0] * 3  # 3 bytes per pixel for RGB

        # Create a simple simulated image (in a real implementation, this would be more complex)
        # For this example, we'll create a gradient image with some noise
        total_pixels = self.camera_resolution[0] * self.camera_resolution[1]
        img_data = []

        for y in range(self.camera_resolution[1]):
            for x in range(self.camera_resolution[0]):
                # Create a gradient based on position
                r = int(128 + 127 * math.sin(x / 50.0))
                g = int(128 + 127 * math.cos(y / 40.0))
                b = int(128 + 127 * math.sin((x + y) / 60.0))

                # Add some noise
                r = min(255, max(0, r + random.randint(-10, 10)))
                g = min(255, max(0, g + random.randint(-10, 10)))
                b = min(255, max(0, b + random.randint(-10, 10)))

                img_data.extend([r, g, b])

        # Convert to bytes
        img_bytes = bytes(img_data)
        img_msg.data = img_bytes

        self.camera_pub.publish(img_msg)

    def publish_visualization_markers(self):
        """Publish visualization markers for RViz"""
        marker_array = MarkerArray()

        # Robot marker
        robot_marker = Marker()
        robot_marker.header.frame_id = "odom"
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = "robot"
        robot_marker.id = 0
        robot_marker.type = Marker.CYLINDER
        robot_marker.action = Marker.ADD
        robot_marker.pose.position = self.robot_pose.position
        robot_marker.pose.orientation = self.robot_pose.orientation
        robot_marker.scale.x = 0.3
        robot_marker.scale.y = 0.3
        robot_marker.scale.z = 0.2
        robot_marker.color.a = 1.0
        robot_marker.color.r = 0.0
        robot_marker.color.g = 0.0
        robot_marker.color.b = 1.0

        marker_array.markers.append(robot_marker)

        # Path marker
        if len(self.pose_history) > 1:
            path_marker = Marker()
            path_marker.header.frame_id = "odom"
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = "path"
            path_marker.id = 1
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD

            # Set scale
            path_marker.scale.x = 0.05  # Line width

            # Set color
            path_marker.color.a = 1.0
            path_marker.color.r = 1.0
            path_marker.color.g = 0.0
            path_marker.color.b = 0.0

            # Set points
            for x, y in self.pose_history:
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.0
                path_marker.points.append(point)

            marker_array.markers.append(path_marker)

        # Environment markers
        for i, obj in enumerate(self.environment_objects):
            env_marker = Marker()
            env_marker.header.frame_id = "odom"
            env_marker.header.stamp = self.get_clock().now().to_msg()
            env_marker.ns = "environment"
            env_marker.id = i + 100  # Start from 100 to avoid conflicts
            env_marker.type = Marker.CUBE if obj['type'] == 'obstacle' else Marker.CUBE
            env_marker.action = Marker.ADD

            env_marker.pose.position.x = obj['position'][0]
            env_marker.pose.position.y = obj['position'][1]
            env_marker.pose.position.z = 0.1  # Slightly above ground
            env_marker.pose.orientation.w = 1.0

            env_marker.scale.x = obj['size'][0]
            env_marker.scale.y = obj['size'][1]
            env_marker.scale.z = 0.2  # Height of 0.2m

            env_marker.color.a = 0.8
            env_marker.color.r = 0.5
            env_marker.color.g = 0.5
            env_marker.color.b = 0.5

            marker_array.markers.append(env_marker)

        self.marker_pub.publish(marker_array)

    def broadcast_transforms(self):
        """Broadcast TF transforms"""
        # Robot base to laser frame
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_frame'

        t.transform.translation.x = 0.1  # 10cm forward
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1  # 10cm up

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

        # Robot base to IMU frame
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.05  # 5cm up

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

        # Robot base to camera frame
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_frame'

        t.transform.translation.x = 0.15  # 15cm forward
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.15  # 15cm up

        # Camera typically points forward, so no rotation needed
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = SimulationIntegrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Simulation integration node shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 2: Create the Launch File

Create the launch file at `~/simulation_ws/src/simulation_integration/simulation_integration/launch/simulation_integration_launch.py`:

```python
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
```

### Step 3: Update Package Configuration

Update the package configuration at `~/simulation_ws/src/simulation_integration/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'simulation_integration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['simulation_integration/launch/simulation_integration_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Complete simulation integration for digital twin systems',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulation_integration_node = simulation_integration.simulation_integration_node:main',
        ],
    },
)
```

### Step 4: Build the Package

Build the ROS 2 package:

```bash
cd ~/simulation_ws
colcon build --packages-select simulation_integration
source install/setup.bash
```

## Running the Complete Simulation System

### Step 1: Launch the Simulation Integration

Launch the complete simulation system:

```bash
cd ~/simulation_ws
source install/setup.bash
ros2 run simulation_integration simulation_integration_node
```

### Step 2: In a new terminal, send velocity commands to control the robot:

```bash
# Terminal 2: Send velocity commands
cd ~/simulation_ws
source install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
```

### Step 3: Monitor the simulation data:

```bash
# Terminal 3: Monitor odometry
ros2 topic echo /odom

# Terminal 4: Monitor laser scan
ros2 topic echo /scan

# Terminal 5: Monitor camera data
ros2 run image_view image_view image:=/camera/image_raw
```

### Step 4: Control the simulation:

```bash
# Pause simulation
ros2 topic pub /sim_control std_msgs/msg/String "data: 'pause'"

# Resume simulation
ros2 topic pub /sim_control std_msgs/msg/String "data: 'resume'"

# Reset simulation
ros2 topic pub /sim_control std_msgs/msg/String "data: 'reset'"

# Change real-time factor to 2x speed
ros2 topic pub /sim_control std_msgs/msg/String "data: 'set_rtf_2.0'"
```

### Step 5: Visualize with RViz2:

```bash
# Terminal 6: Launch RViz2 for visualization
cd ~/simulation_ws
source install/setup.bash
ros2 run rviz2 rviz2
```

In RViz2, add the following displays:
- RobotModel (set Fixed Frame to 'odom')
- LaserScan (set Topic to '/scan')
- Image (set Topic to '/camera/image_raw')
- MarkerArray (set Topic to '/visualization_marker_array')

## Advanced Workflow: Multi-Simulation Coordination

For more advanced scenarios, you can coordinate multiple simulation environments:

### Unity-Style Simulation Node

Create an additional Unity-style simulation node that can work alongside the main integration node:

```python
#!/usr/bin/env python3
"""
Unity-Style Simulation Node

This node simulates Unity-style high-fidelity rendering and physics
that can be coordinated with the main simulation integration node.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
import threading
import time

class UnityStyleSimulationNode(Node):
    def __init__(self):
        super().__init__('unity_style_simulation')

        # Publishers for Unity-style data
        self.high_res_camera_pub = self.create_publisher(Image, '/unity/camera/high_res', 10)
        self.semantic_seg_pub = self.create_publisher(Image, '/unity/semantic_segmentation', 10)
        self.depth_camera_pub = self.create_publisher(Image, '/unity/depth_camera', 10)
        self.physics_state_pub = self.create_publisher(Float32MultiArray, '/unity/physics_state', 10)

        # Subscribers for coordination
        self.sim_state_sub = self.create_subscription(
            PoseStamped, '/sim_state', self.sim_state_callback, 10)

        # Timer for Unity-style simulation update
        self.unity_timer = self.create_timer(0.033, self.unity_simulation_step)  # ~30 FPS

        # Unity-style simulation state
        self.current_pose = None
        self.frame_count = 0

        self.get_logger().info('Unity-style simulation node initialized')

    def sim_state_callback(self, msg):
        """Receive simulation state from main integration node"""
        self.current_pose = msg.pose

    def unity_simulation_step(self):
        """Unity-style simulation update"""
        if self.current_pose is None:
            return

        # Generate high-resolution camera image (simulated)
        self.publish_high_res_camera()

        # Generate semantic segmentation (simulated)
        self.publish_semantic_segmentation()

        # Generate depth camera data (simulated)
        self.publish_depth_camera()

        # Publish physics state
        self.publish_physics_state()

    def publish_high_res_camera(self):
        """Publish high-resolution camera image"""
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'unity_camera_optical_frame'
        img_msg.height = 1080
        img_msg.width = 1920
        img_msg.encoding = 'rgb8'
        img_msg.is_bigendian = 0
        img_msg.step = 1920 * 3  # 3 bytes per pixel

        # Create a more detailed simulated image
        total_pixels = img_msg.width * img_msg.height
        img_data = []

        for y in range(img_msg.height):
            for x in range(img_msg.width):
                # Create a more complex gradient pattern
                r = int(128 + 127 * np.sin(x / 100.0 + self.frame_count * 0.1))
                g = int(128 + 127 * np.cos(y / 80.0 + self.frame_count * 0.1))
                b = int(128 + 127 * np.sin((x + y) / 120.0 + self.frame_count * 0.1))

                # Add some noise for realism
                r = min(255, max(0, r + np.random.randint(-5, 5)))
                g = min(255, max(0, g + np.random.randint(-5, 5)))
                b = min(255, max(0, b + np.random.randint(-5, 5)))

                img_data.extend([r, g, b])

        img_msg.data = bytes(img_data)
        self.high_res_camera_pub.publish(img_msg)

    def publish_semantic_segmentation(self):
        """Publish semantic segmentation data"""
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'unity_camera_optical_frame'
        img_msg.height = 480
        img_msg.width = 640
        img_msg.encoding = 'rgb8'  # Using RGB to encode class information
        img_msg.is_bigendian = 0
        img_msg.step = 640 * 3

        # Simulate semantic segmentation (different colors for different object classes)
        img_data = []
        for y in range(img_msg.height):
            for x in range(img_msg.width):
                # Determine which class this pixel belongs to based on position
                if y < img_msg.height / 4:  # Sky
                    r, g, b = 135, 206, 235  # Sky blue
                elif y > img_msg.height * 0.7:  # Ground
                    r, g, b = 34, 139, 34   # Forest green
                elif (x > img_msg.width * 0.4 and x < img_msg.width * 0.6 and
                      y > img_msg.height * 0.3 and y < img_msg.height * 0.6):  # Robot
                    r, g, b = 255, 255, 255  # White for robot
                else:  # Walls/obstacles
                    r, g, b = 165, 42, 42   # Brown for obstacles

                img_data.extend([r, g, b])

        img_msg.data = bytes(img_data)
        self.semantic_seg_pub.publish(img_msg)

    def publish_depth_camera(self):
        """Publish depth camera data"""
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'unity_depth_camera_frame'
        img_msg.height = 480
        img_msg.width = 640
        img_msg.encoding = '32FC1'  # 32-bit float for depth
        img_msg.is_bigendian = 0
        img_msg.step = 640 * 4  # 4 bytes per float

        # Create depth data (simplified)
        depth_data = []
        for y in range(img_msg.height):
            for x in range(img_msg.width):
                # Calculate distance based on simulated objects
                # For simplicity, create a gradient with some objects
                depth = 10.0  # Default max distance

                # Simulate a wall at distance 3m in the center
                center_x = img_msg.width / 2
                center_y = img_msg.height / 2
                dist_to_center = np.sqrt((x - center_x)**2 + (y - center_y)**2)

                if dist_to_center < 100:  # Circle of radius 100 pixels
                    depth = 3.0  # Wall at 3 meters

                depth_data.append(depth)

        # Convert to bytes (little endian)
        import struct
        depth_bytes = b''
        for d in depth_data:
            depth_bytes += struct.pack('<f', float(d))  # Little endian float

        img_msg.data = depth_bytes
        self.depth_camera_pub.publish(img_msg)

    def publish_physics_state(self):
        """Publish detailed physics state"""
        state_msg = Float32MultiArray()
        state_msg.data = [
            # Position (x, y, z)
            float(self.current_pose.position.x if self.current_pose else 0.0),
            float(self.current_pose.position.y if self.current_pose else 0.0),
            float(self.current_pose.position.z if self.current_pose else 0.0),
            # Orientation (quaternion x, y, z, w)
            float(self.current_pose.orientation.x if self.current_pose else 0.0),
            float(self.current_pose.orientation.y if self.current_pose else 0.0),
            float(self.current_pose.orientation.z if self.current_pose else 0.0),
            float(self.current_pose.orientation.w if self.current_pose else 1.0),
            # Linear velocity (if available)
            0.0, 0.0, 0.0,
            # Angular velocity (if available)
            0.0, 0.0, 0.0,
        ]

        self.physics_state_pub.publish(state_msg)
        self.frame_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = UnityStyleSimulationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Unity-style simulation node shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Educational Applications

### Creating Interactive Simulation Scenarios

For educational purposes, you can create specific simulation scenarios:

1. **Navigation Challenge**: Create a maze environment for path planning
2. **Object Manipulation**: Simulate grasping tasks with Unity-style visuals
3. **Multi-robot Coordination**: Coordinate multiple robots in the same environment
4. **Sensor Fusion**: Combine data from multiple simulated sensors

### Student Exercises

1. **Modify the environment**: Change the layout of obstacles in the simulation
2. **Implement different controllers**: Replace the simple kinematic model with more complex control algorithms
3. **Add new sensors**: Integrate additional sensor types into the simulation
4. **Compare simulation vs. reality**: If you have access to a real robot, compare the simulation results

## Troubleshooting

Common issues and solutions:

1. **High CPU usage**: Reduce simulation frequency or simplify environment
2. **Timing issues**: Ensure all nodes are using the same time source
3. **TF tree problems**: Verify all frame relationships are correctly defined
4. **Data synchronization**: Use appropriate buffer sizes and time synchronization

## Summary

This tutorial provided a comprehensive implementation of a digital twin simulation system that integrates multiple simulation environments with ROS 2. You learned how to create a complete simulation workflow that can be used for educational purposes, research, and development of robotic applications. The system demonstrates the principles of digital twin technology by creating a virtual representation of a physical robot system that can be used for testing, validation, and learning.