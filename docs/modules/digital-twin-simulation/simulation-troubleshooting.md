---
sidebar_position: 5
---

# Simulation Troubleshooting Guide

## Overview

This guide provides solutions for common issues encountered when working with digital twin simulations using Gazebo, Unity-style environments, and ROS 2 integration. The guide covers problems across all simulation components and provides systematic approaches to diagnose and resolve issues.

## Common Issues and Solutions

### 1. Gazebo Simulation Issues

**Problem**: Gazebo fails to start or crashes immediately
**Solutions**:
1. Check graphics drivers and OpenGL support:
   ```bash
   glxinfo | grep "OpenGL renderer"
   ```
2. Verify GPU compatibility and drivers:
   ```bash
   nvidia-smi  # For NVIDIA GPUs
   ```
3. Try running Gazebo with software rendering:
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   gz sim
   ```
4. Check available memory and disk space:
   ```bash
   free -h
   df -h
   ```

**Problem**: Models not loading or displaying incorrectly
**Solutions**:
1. Verify model files exist and have correct permissions:
   ```bash
   ls -la ~/.gazebo/models/
   ```
2. Check model database connectivity:
   ```bash
   gz model --list
   ```
3. Validate SDF files for syntax errors:
   ```bash
   gz sdf -k model.sdf
   ```
4. Clear Gazebo cache if models are corrupted:
   ```bash
   rm -rf ~/.gazebo/models/*
   ```

**Problem**: Physics simulation behaving unrealistically
**Solutions**:
1. Adjust physics parameters in world file:
   ```xml
   <physics type="ode">
     <max_step_size>0.001</max_step_size>
     <real_time_factor>1</real_time_factor>
     <real_time_update_rate>1000</real_time_update_rate>
   </physics>
   ```
2. Check mass and inertia properties of models
3. Verify friction and damping coefficients
4. Use appropriate solver parameters for your simulation

**Problem**: High CPU/GPU usage during simulation
**Solutions**:
1. Reduce physics update rate in world file
2. Simplify collision meshes (use boxes instead of complex meshes)
3. Reduce number of active sensors
4. Lower rendering quality settings
5. Use multi-threaded physics if available

### 2. Unity-Style Simulation Issues

**Problem**: Unity simulation not connecting to ROS 2
**Solutions**:
1. Verify ROS TCP Connector settings:
   - Check IP address and port match ROS 2 network
   - Ensure firewall allows connections on specified port
   - Verify ROS 2 daemon is running: `ros2 daemon status`
2. Check Unity-ROS bridge configuration:
   ```csharp
   ros.Initialize("127.0.0.1", 10000);  // Make sure IP and port match
   ```
3. Confirm Unity build settings include ROS communication

**Problem**: Sensor data not synchronizing between Unity and ROS 2
**Solutions**:
1. Check time synchronization between Unity and ROS 2:
   - Use ROS time instead of Unity time for consistency
   - Implement proper timestamping for sensor messages
2. Verify message rates are appropriate:
   - Camera: typically 15-30 Hz
   - IMU: typically 100-200 Hz
   - Laser scan: typically 10-20 Hz
3. Check for network latency issues

**Problem**: Performance issues in Unity simulation
**Solutions**:
1. Optimize rendering settings:
   - Reduce shadow quality
   - Lower texture resolution
   - Use Level of Detail (LOD) systems
2. Optimize physics settings:
   - Reduce fixed timestep in Unity
   - Use simplified collision meshes
3. Implement occlusion culling and frustum culling
4. Use object pooling for frequently instantiated objects

### 3. ROS 2 Integration Issues

**Problem**: Topics not connecting between simulation components
**Solutions**:
1. Check ROS domain ID consistency:
   ```bash
   echo $ROS_DOMAIN_ID
   ```
2. Verify network configuration if running across multiple machines
3. Check topic names match exactly (case-sensitive):
   ```bash
   ros2 topic list
   ros2 topic info /topic_name
   ```
4. Verify QoS profile compatibility between publishers and subscribers

**Problem**: TF tree errors and coordinate frame issues
**Solutions**:
1. Check TF tree structure:
   ```bash
   ros2 run tf2_tools view_frames
   ```
2. Verify transform broadcaster is running:
   ```bash
   ros2 run tf2_ros tf2_echo map base_link
   ```
3. Check frame ID consistency in all messages
4. Ensure all required transforms are being published

**Problem**: Simulation timing and synchronization issues
**Solutions**:
1. Use simulation time consistently:
   ```bash
   export GAZEBO_USE_SIM_TIME=1
   ros2 param set node_name use_sim_time true
   ```
2. Check real-time factor settings
3. Verify all nodes are using the same time source
4. Implement proper message queuing and buffer management

### 4. Sensor Simulation Issues

**Problem**: Laser scan data not matching expected environment
**Solutions**:
1. Check laser sensor configuration in robot model:
   ```xml
   <sensor name="laser" type="ray">
     <ray>
       <scan>
         <horizontal>
           <samples>720</samples>
           <resolution>1</resolution>
           <min_angle>-1.570796</min_angle>
           <max_angle>1.570796</max_angle>
         </horizontal>
       </scan>
       <range>
         <min>0.1</min>
         <max>30.0</max>
         <resolution>0.01</resolution>
       </range>
     </ray>
   </sensor>
   ```
2. Verify sensor mounting position and orientation
3. Check for sensor noise parameters
4. Validate environment model alignment

**Problem**: Camera images showing incorrect perspective or distortion
**Solutions**:
1. Check camera calibration parameters:
   ```yaml
   camera_name: my_camera
   image_width: 640
   image_height: 480
   distortion_model: plumb_bob
   distortion_coefficients:
     rows: 1
     cols: 5
     data: [0.0, 0.0, 0.0, 0.0, 0.0]
   ```
2. Verify camera intrinsic and extrinsic parameters
3. Check camera mounting position in URDF/SDF
4. Validate image format and encoding

**Problem**: IMU data showing unexpected values
**Solutions**:
1. Check IMU sensor configuration:
   ```xml
   <sensor name="imu" type="imu">
     <always_on>true</always_on>
     <update_rate>100</update_rate>
     <imu>
       <angular_velocity>
         <x>
           <noise type="gaussian">
             <mean>0.0</mean>
             <stddev>0.01</stddev>
           </noise>
         </x>
       </angular_velocity>
     </imu>
   </sensor>
   ```
2. Verify gravity compensation in IMU readings
3. Check for proper sensor mounting and calibration
4. Validate coordinate frame alignment

### 5. Multi-Simulation Coordination Issues

**Problem**: Data synchronization between different simulation environments
**Solutions**:
1. Implement proper time synchronization:
   - Use ROS 2 clock for all simulation components
   - Implement buffer management for different update rates
   - Use interpolation for smooth transitions
2. Check network latency between simulation components
3. Implement proper message sequencing and timestamps
4. Use appropriate QoS settings for real-time communication

**Problem**: State divergence between simulation environments
**Solutions**:
1. Implement state synchronization mechanisms
2. Use consistent physics parameters across environments
3. Implement validation checks between systems
4. Use state estimation and filtering to maintain consistency

## Performance Optimization

### Reducing Computational Load

1. **Physics Optimization**:
   - Use simpler collision geometries
   - Reduce physics update rate where possible
   - Implement spatial partitioning for large environments

2. **Rendering Optimization**:
   - Use Level of Detail (LOD) systems
   - Implement occlusion culling
   - Reduce texture and mesh resolution where appropriate

3. **Communication Optimization**:
   - Compress sensor data when transmitting
   - Use appropriate message rates
   - Implement data filtering and downsampling

### Memory Management

1. **Gazebo Memory Issues**:
   - Monitor simulation memory usage
   - Clear unused models and worlds
   - Use appropriate cache settings

2. **Unity Memory Issues**:
   - Implement object pooling
   - Use asset bundles for large environments
   - Monitor garbage collection patterns

## Debugging Strategies

### Systematic Debugging Approach

1. **Isolate the Problem**:
   - Test each simulation component independently
   - Use minimal configurations to identify issues
   - Create simple test cases

2. **Monitor System Resources**:
   ```bash
   htop  # Monitor CPU usage
   nvidia-smi  # Monitor GPU usage (if applicable)
   df -h  # Check disk space
   ```

3. **Use ROS 2 Tools**:
   ```bash
   ros2 topic echo /topic_name  # Monitor topic data
   ros2 run rqt_graph rqt_graph  # Visualize node connections
   ros2 run rqt_console rqt_console  # Monitor logs
   ```

4. **Enable Detailed Logging**:
   ```bash
   export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG
   ```

### Visualization and Monitoring

1. **RViz2 for ROS 2**:
   - Monitor TF tree
   - Visualize sensor data
   - Display robot models and paths

2. **Gazebo GUI**:
   - Use built-in visualization tools
   - Monitor physics properties
   - Check model states

3. **Custom Monitoring Tools**:
   - Create custom dashboards
   - Implement performance monitoring
   - Log and analyze simulation metrics

## Validation and Testing

### Simulation Validation Steps

1. **Unit Testing**:
   - Test individual simulation components
   - Validate sensor models independently
   - Verify physics parameters

2. **Integration Testing**:
   - Test component interactions
   - Validate data flow between systems
   - Check timing and synchronization

3. **Regression Testing**:
   - Compare against known good configurations
   - Monitor for performance degradation
   - Validate against real-world data when available

### Common Validation Checks

1. **Sensor Data Validation**:
   - Check data ranges and distributions
   - Verify timestamps and frequencies
   - Validate coordinate frame transformations

2. **Physics Validation**:
   - Compare simulated vs. expected behavior
   - Validate conservation laws (momentum, energy)
   - Check for numerical stability

3. **Timing Validation**:
   - Monitor real-time factor
   - Check for dropped messages
   - Validate synchronization accuracy

## Best Practices for Stability

1. **Configuration Management**:
   - Use version control for simulation configurations
   - Document parameter changes and their effects
   - Maintain backup configurations

2. **Error Handling**:
   - Implement graceful degradation
   - Add proper exception handling
   - Provide meaningful error messages

3. **Resource Management**:
   - Set appropriate limits for simulation components
   - Monitor and log resource usage
   - Implement automatic cleanup of unused resources

## Getting Help

If you encounter issues not covered in this guide:

1. Check the [ROS 2 documentation](https://docs.ros.org/)
2. Review the [Gazebo documentation](http://gazebosim.org/)
3. Search the [Unity Robotics documentation](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
4. Use ROS 2 community forums and support channels
5. Consider creating minimal reproducible examples for debugging
6. Monitor system resources and logs to capture the exact state when issues occur