# API Reference

This document provides detailed information about the APIs available in the META-SEJONG AI Robotics Challenge.

## ROS2 Topics

### Camera Topics

#### Fixed Camera Image
```python
Topic: /fixedcam_{x}/rgb/image_raw
Type: sensor_msgs/msg/Image
Description: RGB image from fixed camera {x}
```

#### Fixed Camera Info
```python
Topic: /fixedcam_{x}/camera_info
Type: sensor_msgs/msg/CameraInfo
Description: Camera calibration and pose information
```

### Robot Topics

#### Robot State
```python
Topic: /robot/state
Type: geometry_msgs/msg/PoseStamped
Description: Current robot pose and orientation
```

#### Robot Arm State
```python
Topic: /robot/arm/state
Type: sensor_msgs/msg/JointState
Description: Current joint states of the robotic arm
```

## ROS2 Services

### Navigation Service
```python
Service: /robot/navigate
Type: nav_msgs/srv/GetPlan
Description: Request a path plan to a target location
```

### Pick and Place Service
```python
Service: /robot/pick_and_place
Type: std_srvs/srv/SetBool
Description: Execute pick and place operation
```

## Python API

### Robot Controller Class

```python
class RobotController:
    def __init__(self):
        """Initialize the robot controller."""
        pass

    def detect_objects(self, image):
        """Detect objects in the given image.
        
        Args:
            image (numpy.ndarray): RGB image
            
        Returns:
            list: List of detected objects with locations
        """
        pass

    def plan_path(self, start, goal):
        """Plan a path from start to goal location.
        
        Args:
            start (tuple): Starting coordinates (x, y)
            goal (tuple): Goal coordinates (x, y)
            
        Returns:
            list: List of waypoints
        """
        pass

    def execute_pick_and_place(self, object_location):
        """Execute pick and place operation.
        
        Args:
            object_location (tuple): Object coordinates (x, y, z)
            
        Returns:
            bool: Success status
        """
        pass
```

### Environment Interface

```python
class Environment:
    def __init__(self):
        """Initialize the virtual environment."""
        pass

    def reset(self):
        """Reset the environment to initial state."""
        pass

    def step(self, action):
        """Execute an action in the environment.
        
        Args:
            action (dict): Action to execute
            
        Returns:
            tuple: (observation, reward, done, info)
        """
        pass
```

## Data Structures

### Object Detection Result
```python
class DetectionResult:
    def __init__(self):
        self.class_id: int
        self.confidence: float
        self.bbox: tuple  # (x, y, width, height)
        self.location: tuple  # (x, y, z)
```

### Path Planning Result
```python
class PathResult:
    def __init__(self):
        self.waypoints: list  # List of (x, y) coordinates
        self.cost: float
        self.execution_time: float
```

## Error Handling

### Custom Exceptions
```python
class RobotError(Exception):
    """Base exception for robot-related errors."""
    pass

class NavigationError(RobotError):
    """Exception raised for navigation errors."""
    pass

class ManipulationError(RobotError):
    """Exception raised for manipulation errors."""
    pass
```

## Examples

### Basic Usage
```python
from robot_controller import RobotController
from environment import Environment

# Initialize
env = Environment()
robot = RobotController()

# Detect objects
image = env.get_camera_image()
objects = robot.detect_objects(image)

# Plan and execute
for obj in objects:
    path = robot.plan_path(robot.get_position(), obj.location)
    success = robot.execute_pick_and_place(obj.location)
```

## Support

For additional help:
- Check the [Technical Guide](technical-guide.md)
- Refer to the [FAQ](faq.md)
- Contact the competition organizers 