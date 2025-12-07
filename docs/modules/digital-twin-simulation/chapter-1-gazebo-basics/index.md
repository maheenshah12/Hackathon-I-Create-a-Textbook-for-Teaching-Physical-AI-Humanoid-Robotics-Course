---
sidebar_position: 1
---

# Chapter 1: Gazebo Basics

## Overview

This chapter introduces Gazebo, a powerful robotics simulation environment that enables the creation of realistic digital twins for robotic systems. You'll learn how to set up Gazebo, create simulation environments, spawn robots, and integrate with ROS 2 for testing and validation of robotic applications.

## Learning Objectives

By the end of this chapter, you will be able to:
- Install and configure Gazebo simulation environment
- Create and customize simulation worlds
- Spawn and control robots in simulation
- Integrate Gazebo with ROS 2 for sensor simulation
- Debug and troubleshoot simulation issues
- Understand the principles of digital twin simulation

## Introduction to Gazebo

Gazebo is a 3D simulation environment that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces. It's widely used in robotics research and development as a safe and cost-effective way to test algorithms before deployment on real robots.

### Key Features of Gazebo:
- Physics simulation with multiple engines (ODE, Bullet, Simbody)
- High-quality rendering with support for various sensors
- ROS/ROS 2 integration for robot simulation
- Plugin system for custom functionality
- Model database with thousands of pre-built models

### Gazebo Versions
This chapter focuses on Gazebo Garden (Gazebo 8.x), the latest stable version compatible with ROS 2 Humble. Gazebo Garden provides improved performance, better rendering, and enhanced plugin architecture compared to previous versions.

## Installing Gazebo Garden

### Ubuntu 22.04 Installation

Install Gazebo Garden with the following commands:

```bash
# Add the osrfoundation repository
sudo apt update && sudo apt install wget
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'

# Download and install the osrfoundation key
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Update and install Gazebo Garden
sudo apt update
sudo apt install gz-garden
```

### ROS 2 Integration

To integrate Gazebo with ROS 2 Humble, install the ROS 2 Gazebo packages:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

## Basic Gazebo Concepts

### Worlds
A world file defines the simulation environment, including:
- Physics properties (gravity, magnetic field)
- Models and their initial positions
- Lighting and visual properties
- Plugins and system configurations

World files use the SDF (Simulation Description Format) format:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <!-- Physics properties -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.7 0.7 0.7 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Models
Models represent physical objects in the simulation. They can be:
- Robots with complex kinematics
- Static objects (walls, furniture)
- Sensors and actuators
- Custom objects created for specific applications

Models are defined using SDF format and can include:
- Visual properties (meshes, colors, textures)
- Collision properties (shapes, materials)
- Physics properties (mass, friction)
- Sensors and plugins

### Sensors
Gazebo provides various sensor types:
- **Camera**: RGB, depth, and stereo cameras
- **Lidar**: 2D and 3D LiDAR sensors
- **IMU**: Inertial measurement units
- **Force/Torque**: Joint force and torque sensors
- **GPS**: Global positioning system simulation

## Launching Gazebo

### Basic Launch
Launch Gazebo with the default world:

```bash
gz sim
```

### Launch with Specific World
Launch Gazebo with a custom world file:

```bash
gz sim -r my_world.sdf
```

### Launch with GUI
Launch Gazebo with the graphical interface:

```bash
gz sim -g
```

## Creating a Simple World

Let's create a simple world with a ground plane and a few objects:

1. Create a directory for your world files:
```bash
mkdir -p ~/gazebo_worlds
```

2. Create a simple world file `simple_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Physics -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.7 0.7 0.7 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Simple box -->
    <model name="box">
      <pose>2 2 0.5 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.1 1</ambient>
            <diffuse>0.8 0.2 0.1 1</diffuse>
            <specular>0.8 0.2 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Simple sphere -->
    <model name="sphere">
      <pose>-2 -2 1 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0.1 0.8 0.2 1</ambient>
            <diffuse>0.1 0.8 0.2 1</diffuse>
            <specular>0.1 0.8 0.2 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Working with Models

### Spawning Models
You can spawn models in Gazebo using the command line:

```bash
# Spawn a model at a specific position
gz model -f model.sdf -m my_model_name -x 1.0 -y 2.0 -z 0.5
```

### Using Gazebo Model Database
Gazebo provides a large database of pre-built models. To use a model from the database:

```bash
# List available models
gz model --list

# Spawn a model from the database
gz model -d default -m my_model_name -x 1.0 -y 2.0 -z 0.5
```

## Gazebo and ROS 2 Integration

### ROS 2 Gazebo Packages
The `gazebo_ros_pkgs` package provides integration between Gazebo and ROS 2:

- `gazebo_ros`: Core ROS 2 plugins for Gazebo
- `gazebo_plugins`: Gazebo plugins for various sensors
- `gazebo_msgs`: ROS 2 messages for Gazebo interaction

### Launching Gazebo with ROS 2
To launch Gazebo with ROS 2 integration:

```bash
# Terminal 1: Launch Gazebo
source /opt/ros/humble/setup.bash
gz sim -r -v 4

# Terminal 2: Launch your robot in Gazebo
source /opt/ros/humble/setup.bash
ros2 launch my_robot_gazebo my_robot.launch.py
```

### Example Robot Launch File
Here's an example launch file that spawns a robot in Gazebo:

```python
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch Gazebo server and client
    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=launch.conditions.IfCondition(gui)
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot',
                  '-x', '0.0',
                  '-y', '0.0',
                  '-z', '0.0',
                  '-file', os.path.join(get_package_share_directory('my_robot_description'), 'urdf', 'my_robot.urdf')],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add commands to launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_entity)

    return ld
```

## Basic Simulation Control

### Controlling Simulation
Gazebo provides various ways to control the simulation:

```bash
# Pause simulation
gz service -s /world/default/control --req-type gz.msgs.WorldControl --req 'pause: true'

# Resume simulation
gz service -s /world/default/control --req-type gz.msgs.WorldControl --req 'pause: false'

# Reset simulation
gz service -s /world/default/control --req-type gz.msgs.WorldControl --req 'reset: true'

# Step simulation forward
gz service -s /world/default/control --req-type gz.msgs.WorldControl --req 'step: true'
```

### Getting Model Information
Query model information from Gazebo:

```bash
# Get model poses
gz topic -e /world/default/model/pose/info

# Get model states
gz topic -e /world/default/model/state
```

## Creating a Simple Robot Model

Let's create a simple differential drive robot model:

1. Create a URDF file for the robot:

```xml
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Gazebo plugin for ROS control -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find my_robot_description)/config/my_robot_control.yaml</parameters>
    </plugin>
  </gazebo>
</robot>
```

## Best Practices for Gazebo Simulation

### Performance Optimization
- Use appropriate physics parameters (step size, update rates)
- Limit the number of complex models in the scene
- Use simplified collision meshes when possible
- Consider using GPU acceleration for rendering

### Simulation Accuracy
- Validate physics parameters against real-world measurements
- Test with various environmental conditions
- Compare simulation results with real robot data
- Use appropriate friction and damping coefficients

### Debugging Tips
- Use Gazebo's built-in visualization tools
- Monitor simulation timing and real-time factor
- Check for model collisions and interpenetrations
- Use ROS 2 tools to monitor topic data

## Summary

This chapter introduced the basics of Gazebo simulation environment, including installation, world creation, model spawning, and ROS 2 integration. You learned how to create simple worlds and models, and how to control the simulation. The next chapter will cover more advanced topics including Unity integration for digital twin applications.