# Quickstart Guide: Physical AI Humanoid Robotics Curriculum

## Prerequisites

Before starting this curriculum, ensure you have:

- A computer running Ubuntu 22.04 LTS
- Administrative access to install software
- At least 32GB RAM (64GB recommended)
- A CUDA-compatible GPU with 8GB+ VRAM (RTX series recommended)
- At least 100GB of free disk space
- Internet connection for downloading packages

## Environment Setup

### 1. Install ROS 2 Humble Hawksbill

```bash
# Set locale
locale-gen en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-rosdep2
sudo apt install -y python3-colcon-common-extensions

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install Gazebo

```bash
# Install Gazebo Garden (or Fortress)
sudo apt install ros-humble-gazebo-*
source /opt/ros/humble/setup.bash
```

### 3. Install NVIDIA Isaac Sim

Follow the official installation guide:
1. Download Isaac Sim from NVIDIA Developer website
2. Ensure your system meets the requirements
3. Install Isaac Sim 4.x following the official documentation

### 4. Set up Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## First Tutorial: Creating Your First ROS 2 Node

### 1. Create a Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python beginner_tutorials
```

### 2. Create a Simple Publisher Node

Create the file `~/ros2_ws/src/beginner_tutorials/beginner_tutorials/publisher_member_function.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Update setup.py

In `~/ros2_ws/src/beginner_tutorials/setup.py`, add to the `entry_points` section:

```python
'console_scripts': [
    'talker = beginner_tutorials.publisher_member_function:main',
],
```

### 4. Build and Run

```bash
cd ~/ros2_ws
colcon build --packages-select beginner_tutorials
source install/setup.bash

# In one terminal
ros2 run beginner_tutorials talker
```

## Running Simulations

### 1. Launch a Basic Gazebo Simulation

```bash
# Launch Gazebo
gz sim -v 4

# Or use ROS 2 launch
ros2 launch gazebo_ros empty_world.launch.py
```

### 2. Spawn a Robot Model

```bash
# Spawn a simple robot model
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file path/to/robot/model.urdf
```

## Testing Your Setup

Run this command to verify your ROS 2 installation:

```bash
# Terminal 1
ros2 topic echo /chatter std_msgs/msg/String

# Terminal 2
ros2 topic pub /chatter std_msgs/msg/String "data: Hello, ROS 2!"
```

You should see the message appear in the first terminal.

## Next Steps

1. Proceed to Chapter 1 of the ROS 2 Foundations module
2. Complete the tutorials in sequence
3. Practice with the exercises provided
4. Move on to the simulation module when ready

## Troubleshooting

### Common Issues

- **ROS 2 not found**: Ensure you've sourced the setup.bash file
- **Permission errors**: Run commands with appropriate permissions
- **GPU not detected**: Verify NVIDIA drivers are properly installed
- **Gazebo crashes**: Check GPU drivers and available memory

### Getting Help

- Check the Troubleshooting section in each chapter
- Review the Reference section for common solutions
- Consult the official ROS 2 and Isaac Sim documentation