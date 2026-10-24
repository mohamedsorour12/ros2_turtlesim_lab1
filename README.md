# Alexandria University - Faculty of Engineering
## Design of Mechatronics Systems - Lab 1: ROS 2 & Turtlesim

### Team Members
- **Mohamed Alaaeldin Megahed** - ID: 9591
- **Yousef Mohamed Shaban** - ID: 9579

---

## 1. Package Descriptions
This repository contains two ROS 2 packages:
1. **`turtlesim_interfaces`**: A CMake-based interface package defining:
   - `msg/TargetPose.msg`: Target pose `(x, y, yaw)`.
   - `srv/SetSpeedLimits.srv`: Service to dynamically adjust linear and angular velocity limits.
   - `action/GoToPose.action`: Action to navigate to target pose with live progress feedback.
   - `action/DrawSquare.action`: Higher-level action to reset and draw a complete 4-sided square.
2. **`turtle_controller`**: A Python package implementing:
   - `controller_node`: Subscribes to `/turtle1/pose`, publishes to `/turtle1/cmd_vel`, manages dynamic speed parameters (`max_linear_speed`, `max_angular_speed`), provides the `/set_speed_limits` service, and runs the `goToPose` action server.
   - `reset_service_node`: Provides the `/reset_turtle` service, turning the pen off, teleporting to `(5.54, 5.54, 0.0)`, clearing the screen, and re-enabling the pen without drawing lines.
   - `square_action_server_node`: Coordinates drawing a square using 4 sequential calls to `goToPose` after triggering `/reset_turtle`.
   - `launch/turtle_system.launch.py`: Unified launch file to launch `turtlesim_node` and all custom servers with configurable parameters.

---

## 2. Prerequisites & Installation
- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Desktop
- **Dependencies**:
  ```bash
  sudo apt update
  sudo apt install -y ros-humble-desktop ros-humble-turtlesim python3-colcon-common-extensions ros-dev-tools
