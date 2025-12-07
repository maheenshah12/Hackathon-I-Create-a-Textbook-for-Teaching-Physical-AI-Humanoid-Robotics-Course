#!/usr/bin/env python3
"""
Simple Perception Example for Isaac Sim with ROS 2 Integration

This example demonstrates:
1. Setting up a camera in Isaac Sim
2. Acquiring RGB and depth images
3. Performing basic object detection
4. Publishing data over ROS 2
"""

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.sensor import Camera
from omni.isaac.core.utils.prims import get_prim_at_path
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge


class IsaacSimPerceptionNode(Node):
    def __init__(self):
        super().__init__('isaac_sim_perception_node')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Publishers
        self.rgb_pub = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.object_pose_pub = self.create_publisher(PoseStamped, '/detected_object/pose', 10)

        # Camera parameters
        self.camera_matrix = np.array([
            [616.175, 0.0, 320.5],  # fx, 0, cx
            [0.0, 616.175, 240.5],  # 0, fy, cy
            [0.0, 0.0, 1.0]         # 0, 0, 1
        ])

        # Initialize Isaac Sim components
        self.world = None
        self.camera = None
        self.setup_isaac_sim()

        # Timer for perception loop
        self.timer = self.create_timer(0.1, self.perception_callback)  # 10Hz

    def setup_isaac_sim(self):
        """Set up Isaac Sim environment and camera"""
        print("Setting up Isaac Sim environment...")

        # Create world
        self.world = World(stage_units_in_meters=1.0)

        # Add ground plane
        self.world.scene.add_ground_plane("/World/GroundPlane", static_friction=0.6, dynamic_friction=0.6, restitution=0.1)

        # Add a simple cube to detect
        from omni.isaac.core.objects import DynamicCuboid
        self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Cube",
                name="cube",
                position=np.array([0.5, 0.0, 0.1]),
                size=np.array([0.1, 0.1, 0.1]),
                color=np.array([0.8, 0.1, 0.1])
            )
        )

        # Add camera to the scene
        camera_prim = self.world.scene.add(
            Camera(
                prim_path="/World/Camera",
                name="camera",
                position=np.array([0.0, -0.5, 0.3]),
                frequency=30,
                resolution=(640, 480)
            )
        )

        # Set camera intrinsic parameters
        camera_prim.get_focal_length_attr().set(616.175)
        camera_prim.get_horizontal_aperture_attr().set(34.6)
        camera_prim.get_vertical_aperture_attr().set(25.95)

        # Get camera object
        self.camera = camera_prim

        # Reset the world
        self.world.reset()
        print("Isaac Sim environment setup complete.")

    def perception_callback(self):
        """Main perception callback that runs at 10Hz"""
        if self.world is None or self.camera is None:
            return

        # Step the world to update sensor data
        self.world.step(render=True)

        try:
            # Get RGB image from camera
            rgb_data = self.camera.get_rgb()

            # Get depth image from camera
            depth_data = self.camera.get_depth()

            # Convert to ROS messages
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_data, encoding="bgr8")
            depth_msg = self.bridge.cv2_to_imgmsg(depth_data, encoding="32FC1")

            # Set timestamps
            current_time = self.get_clock().now().to_msg()
            rgb_msg.header.stamp = current_time
            rgb_msg.header.frame_id = "camera_rgb_optical_frame"
            depth_msg.header.stamp = current_time
            depth_msg.header.frame_id = "camera_depth_optical_frame"

            # Publish sensor data
            self.rgb_pub.publish(rgb_msg)
            self.depth_pub.publish(depth_msg)

            # Perform object detection on RGB image
            detected_objects = self.detect_objects(rgb_data, depth_data)

            # Process detected objects
            for obj in detected_objects:
                # Publish object pose
                pose_msg = PoseStamped()
                pose_msg.header.stamp = current_time
                pose_msg.header.frame_id = "map"
                pose_msg.pose.position.x = obj['position'][0]
                pose_msg.pose.position.y = obj['position'][1]
                pose_msg.pose.position.z = obj['position'][2]

                # Simple orientation (pointing toward camera)
                pose_msg.pose.orientation.w = 1.0  # No rotation

                self.object_pose_pub.publish(pose_msg)

        except Exception as e:
            self.get_logger().error(f"Error in perception callback: {str(e)}")

    def detect_objects(self, rgb_image, depth_image):
        """
        Simple object detection using color-based segmentation
        """
        objects = []

        # Convert BGR to HSV for better color segmentation
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

        # Define range for red color (the cube we created)
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        # Create masks for red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # Apply morphological operations to clean up the mask
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Filter by area to avoid small noise
            if cv2.contourArea(contour) > 100:
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)

                # Calculate center of the bounding box in image coordinates
                center_x = x + w // 2
                center_y = y + h // 2

                # Get depth at the center of the object
                depth_value = depth_image[center_y, center_x]

                # Convert pixel coordinates to 3D world coordinates
                if depth_value < 10.0:  # Valid depth
                    # Convert to 3D point using camera intrinsics
                    z_world = depth_value
                    x_world = (center_x - self.camera_matrix[0, 2]) * z_world / self.camera_matrix[0, 0]
                    y_world = (center_y - self.camera_matrix[1, 2]) * z_world / self.camera_matrix[1, 1]

                    objects.append({
                        'position': [x_world, y_world, z_world],
                        'pixel_center': [center_x, center_y],
                        'bbox': [x, y, w, h]
                    })

        return objects


def main(args=None):
    """Main function to run the Isaac Sim perception node"""
    # Initialize Isaac Sim
    print("Initializing Isaac Sim perception system...")

    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the perception node
    perception_node = IsaacSimPerceptionNode()

    print("Isaac Sim perception system initialized. Running perception loop...")

    try:
        # Run the node
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        print("Shutting down perception node...")
    finally:
        # Cleanup
        perception_node.destroy_node()
        rclpy.shutdown()

        # Shutdown Isaac Sim if needed
        print("Perception system shutdown complete.")


if __name__ == '__main__':
    main()