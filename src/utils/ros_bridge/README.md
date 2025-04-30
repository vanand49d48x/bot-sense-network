
# RoboMonitor Bridge for ROS

This package allows any ROS-based robot to connect to the RoboMonitor platform for real-time telemetry monitoring, status tracking, and location visualization.

## Overview

The bridge collects data from standard ROS topics and sends it to the RoboMonitor platform:

- **Battery status** from `/battery_state` topic
- **Temperature** from `/temperature` topic
- **Robot status** from `/robot_status` topic
- **Location** from `/gps/fix` (preferred) or `/odom` topics

## Installation

### Prerequisites

- ROS (Noetic or ROS2)
- Python 3
- `requests` library
- A RoboMonitor account with API key

### Setup Instructions

#### Using Git:

```bash
# Navigate to your ROS workspace src directory
cd ~/catkin_ws/src  # For ROS1
# or
cd ~/ros2_ws/src    # For ROS2

# Clone the repository
git clone https://github.com/robometrics/robometrics-ros-bridge.git robometrics_bridge

# Install dependencies
pip install -r robometrics_bridge/requirements.txt

# Build the workspace
cd ..
catkin_make  # For ROS1
# or
colcon build  # For ROS2
```

#### Alternative Manual Installation:

1. Create a new package in your ROS workspace
2. Copy the files from this repository
3. Install dependencies with `pip install -r requirements.txt`
4. Build your workspace

## Configuration

Configure the bridge using one of these methods:

### 1. Environment Variables

```bash
export ROBOMETRICS_ROBOT_ID="your-robot-id" 
export ROBOMETRICS_API_KEY="your-api-key"
```

### 2. Launch File Parameters

```bash
# ROS1
roslaunch robometrics_bridge robometrics_bridge.launch robot_id:=your-robot-id api_key:=your-api-key

# ROS2
ros2 launch robometrics_bridge robometrics_bridge.launch.py robot_id:=your-robot-id api_key:=your-api-key
```

## Usage

### Start the Bridge (ROS1)

```bash
# Using environment variables
roslaunch robometrics_bridge robometrics_bridge.launch

# Or specify parameters directly
roslaunch robometrics_bridge robometrics_bridge.launch robot_id:=your-robot-id api_key:=your-api-key
```

### Start the Bridge (ROS2)

```bash
# Using environment variables
ros2 launch robometrics_bridge robometrics_bridge.launch.py

# Or specify parameters directly
ros2 launch robometrics_bridge robometrics_bridge.launch.py robot_id:=your-robot-id api_key:=your-api-key
```

## Topic Configuration

The bridge subscribes to the following standard topics:

- `/battery_state` ([sensor_msgs/BatteryState](http://docs.ros.org/en/api/sensor_msgs/html/msg/BatteryState.html))
- `/temperature` ([sensor_msgs/Temperature](http://docs.ros.org/en/api/sensor_msgs/html/msg/Temperature.html))
- `/robot_status` ([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html))
- `/gps/fix` ([sensor_msgs/NavSatFix](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html))
- `/odom` ([nav_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html))

If your robot uses different topic names, you can use ROS topic remapping:

```bash
# ROS1 example
roslaunch robometrics_bridge robometrics_bridge.launch battery_state:=/my_robot/battery temperature:=/my_robot/temp

# ROS2 example
ros2 launch robometrics_bridge robometrics_bridge.launch.py battery_state:=/my_robot/battery temperature:=/my_robot/temp
```

## Troubleshooting

### No data appearing in RoboMonitor dashboard

1. Check that your API key is correct
2. Verify that the topics are publishing data with `rostopic echo`
3. Check for errors in the console output of the bridge node

### Connection errors

If you see connection errors, check:
1. Your internet connection
2. Firewall settings
3. The API URL configuration

## License

MIT License
