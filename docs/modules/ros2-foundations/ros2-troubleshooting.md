---
sidebar_position: 6
---

# ROS 2 Troubleshooting Guide

## Common Issues and Solutions

### 1. Environment Not Sourced

**Problem**: ROS 2 commands are not recognized or "command not found" errors.

**Solution**:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

To make this permanent, add these lines to your `~/.bashrc`:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 2. Package Not Found

**Problem**: When running `ros2 run package_name executable_name`, you get "Package 'package_name' not found".

**Solution**:
1. Ensure the package is built: `cd ~/ros2_ws && colcon build --packages-select package_name`
2. Source the workspace: `source install/setup.bash`
3. Check if the package exists in your workspace: `ls src/`

### 3. Topic Communication Issues

**Problem**: Publisher and subscriber nodes cannot communicate.

**Solutions**:
1. Verify topic names match exactly (case-sensitive): `ros2 topic list`
2. Check if nodes are running: `ros2 node list`
3. Verify message types match: `ros2 topic info /topic_name`
4. Check for firewall issues if nodes run on different machines

### 4. Import Errors in Python Nodes

**Problem**: "ModuleNotFoundError" when importing ROS 2 libraries.

**Solutions**:
1. Ensure you're using the correct Python environment: `which python3`
2. Install ROS 2 Python packages: `pip3 install rclpy std_msgs`
3. Make sure your package.xml has the correct dependencies

### 5. DDS Implementation Issues

**Problem**: Nodes on different machines cannot communicate, or experiencing communication delays.

**Solutions**:
1. Check DDS configuration: `echo $RMW_IMPLEMENTATION`
2. For cross-platform communication, ensure all systems use the same DDS implementation
3. Check network configuration and firewall settings

### 6. Permission Errors

**Problem**: Cannot create or modify workspace files.

**Solutions**:
1. Ensure proper ownership: `sudo chown -R $USER:$USER ~/ros2_ws`
2. Check file permissions: `ls -la ~/ros2_ws`
3. Avoid running ROS 2 commands with sudo unless absolutely necessary

### 7. Build Errors

**Problem**: `colcon build` fails with compilation errors.

**Solutions**:
1. Install missing dependencies: `rosdep install --from-paths src --ignore-src -r -y`
2. Check for syntax errors in package.xml and setup.py
3. Clean build directory: `rm -rf build/ install/ log/`
4. Try building a single package: `colcon build --packages-select package_name`

### 8. Memory Issues

**Problem**: System becomes unresponsive during builds or when running multiple nodes.

**Solutions**:
1. Limit parallel builds: `colcon build --parallel-workers 2`
2. Add swap space if RAM is limited
3. Close unnecessary applications during builds

### 9. Time Synchronization

**Problem**: Nodes show time-related warnings or actions behave unexpectedly.

**Solutions**:
1. Ensure system time is synchronized: `timedatectl status`
2. For distributed systems, use NTP or PTP for time synchronization
3. Check if `ros2 topic echo` shows timestamps advancing properly

### 10. Parameter Issues

**Problem**: Parameters not being set or retrieved correctly.

**Solutions**:
1. Ensure parameters are declared before use: `self.declare_parameter('param_name', default_value)`
2. Check parameter names for typos
3. Verify parameter files have correct format

## Debugging Strategies

### Using ROS 2 Tools

1. **Check active nodes**: `ros2 node list`
2. **Inspect topics**: `ros2 topic list` and `ros2 topic echo /topic_name`
3. **Check services**: `ros2 service list`
4. **Monitor actions**: `ros2 action list`

### Logging

1. Use appropriate log levels:
   - `self.get_logger().debug()` for detailed debugging
   - `self.get_logger().info()` for general information
   - `self.get_logger().warn()` for warnings
   - `self.get_logger().error()` for errors
   - `self.get_logger().fatal()` for fatal errors

2. Enable logging: `export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG`

### Common Debugging Commands

```bash
# List all topics and their types
ros2 topic list -t

# Echo messages from a topic
ros2 topic echo /topic_name std_msgs/msg/String

# Call a service
ros2 service call /service_name service_type "{request_field: value}"

# Check node graph
ros2 run rqt_graph rqt_graph
```

## Best Practices for Avoiding Issues

1. Always source your environment before working with ROS 2
2. Use virtual environments for Python development
3. Test individual components before integrating
4. Use meaningful names for nodes, topics, and services
5. Include error handling in your nodes
6. Document your code and configurations
7. Regularly update and maintain your workspace

## Getting Help

If you encounter an issue not covered here:

1. Check the official ROS 2 documentation
2. Search ROS Answers: https://answers.ros.org/
3. Check the ROS Discourse forum
4. Look for similar issues on GitHub repositories
5. Ask specific questions with detailed error messages and context