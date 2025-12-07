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