# Data Model: Physical AI Humanoid Robotics Curriculum

## Curriculum Module
- **name**: String (required) - Module name (e.g., "ROS2 Foundations", "Digital Twin", "NVIDIA Isaac", "VLA Pipelines")
- **description**: String (required) - Brief description of module content
- **chapters**: List<Chapter> (required) - List of chapters in the module
- **learning_objectives**: List<String> (required) - List of learning objectives for the module
- **prerequisites**: List<String> (optional) - Prerequisites for the module
- **estimated_duration**: Integer (required) - Estimated duration in hours

## Chapter
- **title**: String (required) - Chapter title
- **objective**: String (required) - Learning objective for the chapter
- **content**: String (required) - Main content of the chapter
- **tutorials**: List<Tutorial> (required) - List of hands-on tutorials in the chapter
- **code_examples**: List<CodeExample> (required) - List of code examples in the chapter
- **exercises**: List<Exercise> (optional) - List of exercises for the chapter
- **summary**: String (required) - Chapter summary
- **troubleshooting**: List<String> (required) - Common errors and troubleshooting tips

## Tutorial
- **title**: String (required) - Tutorial title
- **description**: String (required) - Brief description of the tutorial
- **steps**: List<String> (required) - Step-by-step instructions
- **expected_outcome**: String (required) - What the student should achieve
- **prerequisites**: List<String> (optional) - Prerequisites for the tutorial
- **estimated_time**: Integer (required) - Estimated time in minutes

## Code Example
- **title**: String (required) - Example title
- **language**: String (required) - Programming language (e.g., "Python", "C++")
- **code**: String (required) - The actual code
- **description**: String (required) - What the code does
- **dependencies**: List<String> (required) - ROS packages or other dependencies needed
- **tested_environment**: String (required) - Environment where code was tested (e.g., "Ubuntu 22.04, ROS 2 Humble")
- **expected_output**: String (optional) - Expected output when running the code

## Exercise
- **title**: String (required) - Exercise title
- **description**: String (required) - Detailed description of the exercise
- **difficulty**: Enum (required) - Difficulty level ("beginner", "intermediate", "advanced")
- **expected_outcome**: String (required) - What the student should achieve
- **evaluation_criteria**: List<String> (required) - How the exercise will be evaluated

## Simulation Environment
- **name**: String (required) - Name of the simulation environment (e.g., "Gazebo Fortress", "Isaac Sim 4.x", "Unity")
- **version**: String (required) - Version of the simulation environment
- **configuration**: String (required) - Configuration details and setup instructions
- **robot_models**: List<String> (required) - List of robot models supported
- **capabilities**: List<String> (required) - Simulation capabilities (physics, sensors, etc.)

## VLA Pipeline
- **name**: String (required) - Name of the VLA pipeline
- **input_type**: String (required) - Type of input (e.g., "text", "speech")
- **processing_steps**: List<String> (required) - Steps in the processing pipeline
- **output_type**: String (required) - Type of output (e.g., "robot action", "motor command")
- **components**: List<String> (required) - Components used (e.g., "Whisper", "GPT", "ROS action server")
- **integration_points**: List<String> (required) - Points where the pipeline integrates with other systems

## Hardware Deployment Configuration
- **target_platform**: String (required) - Target hardware platform (e.g., "Jetson Orin NX", "Jetson Orin Nano")
- **requirements**: String (required) - Hardware requirements
- **setup_steps**: List<String> (required) - Steps to set up the hardware
- **deployment_method**: String (required) - How to deploy code to the hardware
- **troubleshooting**: List<String> (required) - Common deployment issues and solutions

## Capstone Project
- **title**: String (required) - Capstone project title
- **description**: String (required) - Detailed description of the project
- **objectives**: List<String> (required) - Learning objectives for the project
- **components**: List<String> (required) - Components required for the project
- **milestones**: List<Milestone> (required) - Project milestones
- **evaluation_criteria**: List<String> (required) - How the project will be evaluated

## Milestone
- **title**: String (required) - Milestone title
- **description**: String (required) - Description of what should be completed
- **deliverables**: List<String> (required) - Deliverables for the milestone
- **success_criteria**: List<String> (required) - Criteria for milestone completion
- **estimated_duration**: Integer (required) - Estimated time in hours