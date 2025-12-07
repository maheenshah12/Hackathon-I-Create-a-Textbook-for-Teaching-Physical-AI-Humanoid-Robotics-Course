# Research: Physical AI Humanoid Robotics Curriculum

## ROS 2 Version Decision

**Decision**: ROS 2 Humble Hawksbill (recommended)
**Rationale**:
- Long-term support (LTS) release with 5-year support cycle (2022-2027)
- Stable and well-documented
- Better compatibility with NVIDIA Isaac Sim
- Recommended for production robotics applications
- More learning resources available due to stability

**Alternatives considered**:
- ROS 2 Iron Irwini: Newer but shorter support cycle, less stable for educational content
- ROS 2 Rolling: Cutting edge but not suitable for curriculum due to frequent changes

## Isaac Sim Version and System Requirements

**Decision**: Isaac Sim 4.x with RTX GPU recommended but CUDA-compatible GPU minimum
**Rationale**:
- Isaac Sim 4.x is the latest stable version with extensive documentation
- RTX GPUs provide optimal performance for perception tasks and synthetic data generation
- Minimum requirement: CUDA-compatible GPU with 8GB+ VRAM
- System requirements: Ubuntu 20.04/22.04, 32GB RAM, 100GB+ disk space

**Alternatives considered**:
- Isaac Sim 3.x: Older version, less features
- Other simulation platforms: Less integration with NVIDIA tools

## Simulation Platform Choice

**Decision**: Gazebo for core simulation, with Isaac Sim for perception tasks
**Rationale**:
- Gazebo Fortress/Garden provides excellent physics simulation for humanoid robots
- Isaac Sim specializes in perception, VSLAM, and synthetic data generation
- Complementary tools rather than competing options
- Both well-supported and documented

**Alternatives considered**:
- Unity: Good for visualization but less robotics-focused
- Other simulators: Less integration with ROS 2

## Edge Hardware for Capstone

**Decision**: NVIDIA Jetson Orin NX (recommended) vs Orin Nano (minimum)
**Rationale**:
- Orin NX: Better performance for VLA pipelines, more memory (16GB)
- Orin Nano: Cost-effective option with sufficient capability for basic tasks
- Both support ROS 2 Humble and Isaac Sim containers
- Clear tiered recommendations for different budgets/scope

**Alternatives considered**:
- Other SBCs: Less ROS 2 support or performance
- Cloud deployment: Not suitable for physical robot control

## Capstone Approach

**Decision**: Full humanoid robot simulation with proxy hardware option
**Rationale**:
- Start with simulation to ensure accessibility for all students
- Provide clear path to physical hardware for those who have it
- Use standard humanoid models (e.g., ROS-Industrial) for consistency
- Proxy robot option for institutions without full humanoid robots

**Alternatives considered**:
- Simplified mobile robot: Less relevant to humanoid focus
- Only physical hardware: Limits accessibility

## Code Testing Methodology

**Decision**: Local simulation with verification steps and automated tests
**Rationale**:
- Local testing ensures examples work in student environments
- Verification steps: Build, launch, basic functionality check
- Automated tests: Basic syntax and dependency checks
- Cloud simulation as supplementary option for complex scenarios
- Documentation of expected outputs for each example

**Alternatives considered**:
- Cloud-only testing: Less reliable for curriculum
- Manual-only testing: Higher error rate