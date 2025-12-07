---
sidebar_position: 9
---

# Chapter 3: Manipulation

## Objective

By the end of this chapter, you will understand robotic manipulation principles, how to simulate manipulation tasks in Isaac Sim, and how to implement perception-driven manipulation systems that integrate visual feedback with robotic control.

## Introduction to Robotic Manipulation

Robotic manipulation involves the control of robotic end-effectors to interact with objects in the environment. This includes:
- **Grasping**: Picking up objects
- **Transport**: Moving objects from one location to another
- **Placement**: Accurately placing objects
- **Assembly**: Manipulating objects to build structures

## Manipulation Pipeline

A typical manipulation pipeline consists of:
1. **Perception**: Detecting and localizing objects
2. **Planning**: Computing grasp poses and trajectories
3. **Control**: Executing manipulation actions
4. **Feedback**: Adjusting based on sensor data

## Object Detection and Pose Estimation

### 3D Object Detection in Isaac Sim
```python
import omni
from omni.isaac.core.utils.prims import get_prim_at_path
from omvi.isaac.core.utils.bounds import compute_viewport_params
import numpy as np

class ObjectDetector:
    def __init__(self):
        self.camera = None
        self.detector = None

    def detect_objects(self, camera_view):
        """
        Detect objects in the camera view
        Returns: List of object poses and types
        """
        # Get segmentation data from Isaac Sim
        segmentation = self.get_segmentation_data()

        # Process segmentation to find objects
        objects = self.process_segmentation(segmentation)

        # Estimate 3D poses from 2D detections
        object_poses = self.estimate_poses(objects, camera_view)

        return object_poses

    def get_segmentation_data(self):
        """
        Get segmentation data from Isaac Sim
        """
        from omni.isaac.core.utils.semantics import get_semantic_bounding_boxes
        return get_semantic_bounding_boxes("/World", "class")
```

### Pose Estimation
```python
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

def estimate_object_pose(keypoints_2d, object_model_3d, camera_matrix):
    """
    Estimate 3D pose of object using PnP algorithm
    """
    # Solve Perspective-n-Point problem
    success, rvec, tvec = cv2.solvePnP(
        object_model_3d,
        keypoints_2d,
        camera_matrix,
        None  # Distortion coefficients
    )

    if success:
        # Convert rotation vector to rotation matrix
        rotation_matrix, _ = cv2.Rodrigues(rvec)

        # Create transformation matrix
        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = tvec.flatten()

        return transform
    else:
        return None
```

## Grasp Planning

### Grasp Pose Generation
```python
class GraspPlanner:
    def __init__(self):
        self.grasp_database = self.load_grasp_database()

    def generate_grasps(self, object_pose, object_type):
        """
        Generate potential grasp poses for an object
        """
        # Load object-specific grasp poses
        if object_type in self.grasp_database:
            grasp_poses = self.grasp_database[object_type]
        else:
            # Generate generic grasps
            grasp_poses = self.generate_generic_grasps(object_pose)

        # Transform grasps to world frame
        world_grasps = []
        for grasp in grasp_poses:
            world_grasp = object_pose @ grasp
            world_grasps.append(world_grasp)

        return world_grasps

    def evaluate_grasps(self, grasp_poses, scene_data):
        """
        Evaluate grasp quality based on scene constraints
        """
        scores = []
        for grasp in grasp_poses:
            score = self.calculate_grasp_score(grasp, scene_data)
            scores.append(score)

        return scores
```

### Antipodal Grasp Detection
```python
def find_antipodal_grasps(point_cloud, normals):
    """
    Find potential antipodal grasp points from point cloud
    """
    grasps = []

    # Find pairs of points with opposing normals
    for i in range(len(point_cloud)):
        for j in range(i+1, len(point_cloud)):
            # Check if normals are roughly opposing
            dot_product = np.dot(normals[i], normals[j])

            if dot_product < -0.8:  # Opposing normals
                # Check distance is within gripper range
                distance = np.linalg.norm(point_cloud[i] - point_cloud[j])

                if 0.02 < distance < 0.1:  # Gripper width constraints
                    # Calculate grasp pose
                    grasp_center = (point_cloud[i] + point_cloud[j]) / 2
                    grasp_axis = point_cloud[j] - point_cloud[i]
                    grasp_axis = grasp_axis / np.linalg.norm(grasp_axis)

                    # Create grasp orientation
                    z_axis = grasp_axis
                    x_axis = np.array([1, 0, 0])
                    y_axis = np.cross(z_axis, x_axis)
                    y_axis = y_axis / np.linalg.norm(y_axis)
                    x_axis = np.cross(y_axis, z_axis)

                    grasp_pose = np.eye(4)
                    grasp_pose[:3, :3] = np.column_stack([x_axis, y_axis, z_axis])
                    grasp_pose[:3, 3] = grasp_center

                    grasps.append(grasp_pose)

    return grasps
```

## Manipulation Control

### Joint Space Control
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ManipulationController(Node):
    def __init__(self):
        super().__init__('manipulation_controller')

        # Publishers for different control interfaces
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Robot parameters
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.home_pose = [0.0, -1.0, 0.0, -2.0, 0.0, 0.0]

    def move_to_pose(self, joint_positions, duration=5.0):
        """
        Move robot to specified joint positions
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        trajectory.points.append(point)

        self.trajectory_pub.publish(trajectory)

    def compute_ik(self, target_pose):
        """
        Compute inverse kinematics for target pose
        """
        # This would interface with an IK solver
        # For example, using KDL, MoveIt, or custom solver
        joint_positions = self.ik_solver(target_pose)
        return joint_positions

    def grasp_object(self, object_pose):
        """
        Execute grasping sequence
        """
        # 1. Move to pre-grasp position
        pre_grasp = object_pose.copy()
        pre_grasp[2, 3] += 0.1  # 10cm above object

        joint_positions = self.compute_ik(pre_grasp)
        self.move_to_pose(joint_positions)

        # 2. Move to grasp position
        joint_positions = self.compute_ik(object_pose)
        self.move_to_pose(joint_positions)

        # 3. Close gripper
        self.close_gripper()

        # 4. Lift object
        lift_pose = object_pose.copy()
        lift_pose[2, 3] += 0.1  # Lift 10cm
        joint_positions = self.compute_ik(lift_pose)
        self.move_to_pose(joint_positions)
```

### Cartesian Control
```python
def cartesian_control(robot_model, target_pose, current_joints, dt=0.01):
    """
    Perform Cartesian space control with Jacobian transpose
    """
    # Compute forward kinematics
    current_pose = robot_model.forward_kinematics(current_joints)

    # Compute pose error
    position_error = target_pose[:3, 3] - current_pose[:3, 3]
    rotation_error = rotation_error(target_pose[:3, :3], current_pose[:3, :3])

    # Combine position and rotation errors
    pose_error = np.concatenate([position_error, rotation_error])

    # Compute Jacobian
    jacobian = robot_model.jacobian(current_joints)

    # Compute joint velocities using Jacobian transpose
    joint_velocities = dt * jacobian.T @ pose_error

    # Update joint positions
    new_joints = current_joints + joint_velocities

    return new_joints

def rotation_error(R1, R2):
    """
    Compute rotation error between two rotation matrices
    """
    R_rel = R1.T @ R2
    angle_axis = rotation_matrix_to_angle_axis(R_rel)
    return angle_axis
```

## Isaac Sim Manipulation Setup

### Creating a Manipulation Scene
```python
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

def setup_manipulation_scene():
    """
    Set up a manipulation scene in Isaac Sim
    """
    world = World(stage_units_in_meters=1.0)

    # Add robot
    assets_root_path = get_assets_root_path()
    robot_path = assets_root_path + "/Isaac/Robots/Franka/franka_instanceable.usd"
    add_reference_to_stage(robot_path, prim_path="/World/Robot")

    # Add table
    table = world.scene.add(
        DynamicCuboid(
            prim_path="/World/Table",
            name="table",
            position=np.array([0.5, 0, 0.5]),
            size=np.array([0.8, 1.2, 0.05]),
            color=np.array([0.8, 0.8, 0.8])
        )
    )

    # Add objects to manipulate
    object1 = world.scene.add(
        DynamicCuboid(
            prim_path="/World/Object1",
            name="object1",
            position=np.array([0.5, 0, 0.6]),
            size=np.array([0.05, 0.05, 0.05]),
            color=np.array([1.0, 0, 0])
        )
    )

    # Reset the world
    world.reset()

    return world
```

## Perception-Action Integration

### Closed-Loop Manipulation
```python
class PerceptionActionNode(Node):
    def __init__(self):
        super().__init__('perception_action_node')

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.image_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10
        )

        # Publishers
        self.command_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )

        # State variables
        self.latest_image = None
        self.latest_depth = None
        self.object_detected = False
        self.object_pose = None

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def image_callback(self, msg):
        self.latest_image = msg

    def depth_callback(self, msg):
        self.latest_depth = msg

    def control_loop(self):
        if self.latest_image is not None and self.latest_depth is not None:
            # Detect object
            object_pose = self.detect_object(
                self.latest_image,
                self.latest_depth
            )

            if object_pose is not None:
                # Plan and execute grasp
                self.execute_grasp(object_pose)
            else:
                # Search for object
                self.search_for_object()
```

## Force Control and Compliance

### Force/Torque Sensing
```python
from geometry_msgs.msg import WrenchStamped

class ForceControlNode(Node):
    def __init__(self):
        super().__init__('force_control_node')

        self.force_sub = self.create_subscription(
            WrenchStamped, '/ft_sensor/wrench', self.force_callback, 10
        )

        self.contact_force_threshold = 5.0  # Newtons
        self.contact_normal_threshold = 0.5  # Normal force threshold

    def force_callback(self, msg):
        self.current_force = msg.wrench
        self.check_contact()

    def check_contact(self):
        force_magnitude = np.sqrt(
            self.current_force.force.x**2 +
            self.current_force.force.y**2 +
            self.current_force.force.z**2
        )

        if force_magnitude > self.contact_force_threshold:
            self.get_logger().info('Contact detected!')
            # Adjust control strategy for contact tasks
```

## Summary

In this chapter, you've learned about robotic manipulation principles, including object detection, grasp planning, and manipulation control. You now understand how to create perception-driven manipulation systems that integrate visual feedback with robotic control, and how to simulate these systems in Isaac Sim.

## Common Errors and Troubleshooting

- **Grasp failures**: Ensure proper grasp planning and sufficient friction models
- **Collision issues**: Use proper collision checking and avoid singularities
- **Force control instability**: Implement proper force control gains and filtering
- **Perception errors**: Validate object detection pipeline and handle occlusions

## References

- [ROS Manipulation Tutorials](http://wiki.ros.org/manipulation)
- [MoveIt Documentation](https://moveit.ros.org/)
- [Isaac Sim Manipulation Examples](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_manipulation.html)
- [Grasp Planning Survey](https://arxiv.org/abs/1908.04822)