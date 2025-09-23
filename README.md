# TurtleBot Behavior Tree Emergency Stop Demo

This package demonstrates behavior tree control of a TurtleBot running ROS 2, using the Nav2 navigation stack. The robot automatically stops navigation if a human is detected lying down, as indicated by the `/posture_status` topic.

## Features

- **Behavior Tree Navigation**: The robot navigates between two preset points using a simple behavior tree.
- **Emergency Stop**: If a message with value `1` (LYING DOWN) arrives at the `/posture_status` topic, all motion and navigation are cancelled immediately.

## Prerequisites

- **Operating System**: Ubuntu 22.04 recommended
- **ROS 2 Version**: Humble or newer
- **TurtleBot hardware or simulation (optional)**

## Dependencies

Ensure these ROS 2 packages are installed:
- `rclcpp`
- `rclcpp_action`
- `nav2_msgs`
- `behaviortree_cpp_v3`
- `geometry_msgs`
- `std_msgs`
- `ament_cmake`

Dependencies are managed via package manifests.

## Installation

Clone the repository into your ROS 2 workspace source folder:

```
cd ~/ros2_ws/src
```

Build the package:

```
cd ~/ros2_ws
colcon build --packages-select turtlebot_bt
```

Source the workspace:

```
source ~/ros2_ws/install/setup.bash
```

## Usage

To run the executable:

```
ros2 run turtlebot_bt executable
```

This launches:
- The main behavior tree executor
- Subscribers for Nav2 navigation status
- A subscriber for `/posture_status` (std_msgs/msg/Int32) topic

### Emergency Stop Functionality

**Topic:** `/posture_status`  
**Message Type:** `std_msgs/msg/Int32`  
**Trigger:** If a message with data `1` is received, the robot immediately stops navigation.

Test by publishing:

```
ros2 topic pub /posture_status std_msgs/msg/Int32 "{data: 1}"
```



## Customization

- Adjust navigation waypoints in `bt_xml/bt_tree_groot.xml`.
- Extend the behavior tree as needed.

## Troubleshooting

- Ensure Nav2 action server is running.
- Check topic names and message types match.
- Confirm dependencies are correctly installed.



---

For questions or contributions, feel free to open issues or pull requests.
```
Tip: For best results, first run the Nav2 bringup demo (or turtlebot bringup), launch navigation, and then execute this package. The behavior tree will stop the robot even if a navigation is in progress when /posture_status is set to 1.
