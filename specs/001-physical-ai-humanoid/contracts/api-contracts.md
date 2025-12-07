# API Contracts: Physical AI Humanoid Robotics Curriculum

## Curriculum Content API

### Get Module Information
- **Endpoint**: `GET /api/modules/{module_id}`
- **Description**: Retrieve information about a specific curriculum module
- **Path Parameters**:
  - `module_id`: String - ID of the module (e.g., "ros2-foundations", "digital-twin-simulation", "nvidia-isaac-perception", "vla-pipelines")
- **Response**:
  - **200 OK**: Module information object
    ```json
    {
      "id": "string",
      "name": "string",
      "description": "string",
      "chapters": [
        {
          "id": "string",
          "title": "string",
          "duration_minutes": "integer",
          "objectives": ["string"]
        }
      ],
      "duration_hours": "integer",
      "prerequisites": ["string"]
    }
    ```
  - **404 Not Found**: Module does not exist

### Get Chapter Content
- **Endpoint**: `GET /api/modules/{module_id}/chapters/{chapter_id}`
- **Description**: Retrieve the content of a specific chapter
- **Path Parameters**:
  - `module_id`: String - ID of the module
  - `chapter_id`: String - ID of the chapter
- **Response**:
  - **200 OK**: Chapter content object
    ```json
    {
      "id": "string",
      "title": "string",
      "objective": "string",
      "content": "string",
      "tutorials": [
        {
          "title": "string",
          "steps": ["string"],
          "expected_outcome": "string",
          "estimated_time": "integer"
        }
      ],
      "code_examples": [
        {
          "title": "string",
          "language": "string",
          "code": "string",
          "description": "string"
        }
      ],
      "exercises": [
        {
          "title": "string",
          "description": "string",
          "difficulty": "enum: beginner|intermediate|advanced"
        }
      ],
      "summary": "string",
      "troubleshooting": ["string"]
    }
    ```
  - **404 Not Found**: Module or chapter does not exist

### Get Tutorial Steps
- **Endpoint**: `GET /api/modules/{module_id}/chapters/{chapter_id}/tutorials/{tutorial_id}`
- **Description**: Retrieve detailed steps for a specific tutorial
- **Path Parameters**:
  - `module_id`: String - ID of the module
  - `chapter_id`: String - ID of the chapter
  - `tutorial_id`: String - ID of the tutorial
- **Response**:
  - **200 OK**: Tutorial steps object
    ```json
    {
      "id": "string",
      "title": "string",
      "description": "string",
      "steps": [
        {
          "step_number": "integer",
          "description": "string",
          "code_example": "string",
          "expected_result": "string"
        }
      ],
      "prerequisites": ["string"],
      "estimated_time": "integer"
    }
    ```
  - **404 Not Found**: Module, chapter, or tutorial does not exist

### Execute Code Example
- **Endpoint**: `POST /api/code-examples/execute`
- **Description**: Execute a code example in a simulation environment
- **Request Body**:
  ```json
  {
    "code": "string",
    "language": "string",
    "environment": "string",
    "simulation_config": "object"
  }
  ```
- **Response**:
  - **200 OK**: Execution result
    ```json
    {
      "success": "boolean",
      "output": "string",
      "execution_time": "number",
      "errors": ["string"]
    }
    ```
  - **400 Bad Request**: Invalid request parameters

### Get Hardware Deployment Instructions
- **Endpoint**: `GET /api/deployment/{hardware_type}`
- **Description**: Retrieve deployment instructions for specific hardware
- **Path Parameters**:
  - `hardware_type`: String - Type of hardware (e.g., "jetson-orin-nx", "jetson-orin-nano")
- **Response**:
  - **200 OK**: Deployment instructions object
    ```json
    {
      "hardware_type": "string",
      "requirements": {
        "cpu": "string",
        "memory": "string",
        "storage": "string",
        "gpu": "string"
      },
      "setup_steps": ["string"],
      "deployment_method": "string",
      "troubleshooting": ["string"]
    }
    ```
  - **404 Not Found**: Hardware type not supported

## Simulation Environment API

### Launch Simulation
- **Endpoint**: `POST /api/simulation/launch`
- **Description**: Launch a simulation environment
- **Request Body**:
  ```json
  {
    "simulation_type": "enum: gazebo|isaac|unity",
    "robot_model": "string",
    "world_file": "string",
    "configuration": "object"
  }
  ```
- **Response**:
  - **200 OK**: Simulation launched successfully
    ```json
    {
      "simulation_id": "string",
      "status": "string",
      "connection_info": "object"
    }
    ```
  - **400 Bad Request**: Invalid simulation parameters

### Get Simulation Status
- **Endpoint**: `GET /api/simulation/{simulation_id}/status`
- **Description**: Get the status of a running simulation
- **Path Parameters**:
  - `simulation_id`: String - ID of the simulation
- **Response**:
  - **200 OK**: Simulation status
    ```json
    {
      "simulation_id": "string",
      "status": "enum: running|paused|stopped|error",
      "robot_status": "object",
      "simulation_time": "number"
    }
    ```
  - **404 Not Found**: Simulation does not exist

### Execute VLA Pipeline
- **Endpoint**: `POST /api/vla/execute`
- **Description**: Execute a Vision-Language-Action pipeline
- **Request Body**:
  ```json
  {
    "input_text": "string",
    "components": {
      "speech_recognition": "string",
      "language_model": "string",
      "action_mapping": "string"
    },
    "robot_config": "object"
  }
  ```
- **Response**:
  - **200 OK**: Pipeline execution result
    ```json
    {
      "success": "boolean",
      "actions": ["string"],
      "execution_time": "number",
      "confidence_scores": ["number"]
    }
    ```
  - **400 Bad Request**: Invalid pipeline parameters