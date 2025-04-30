
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
- `requests` library (for HTTP)
- `paho-mqtt` library (for MQTT)
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

#### Using rosdep (Recommended):

```bash
# Add the repository to your sources list
sudo sh -c 'echo "deb http://packages.robometrics.xyz/apt $(lsb_release -sc) main" > /etc/apt/sources.list.d/robometrics.list'
curl -sL https://packages.robometrics.xyz/key.gpg | sudo apt-key add -
sudo apt update

# Install the package
sudo apt install ros-$ROS_DISTRO-robometrics-bridge

# For ROS2, replace $ROS_DISTRO with your ROS2 distribution (e.g. foxy, humble)
# sudo apt install ros-humble-robometrics-bridge
```

## Configuration

Configure the bridge using one of these methods:

### 1. Environment Variables

```bash
export ROBOMETRICS_ROBOT_ID="your-robot-id" 
export ROBOMETRICS_API_KEY="your-api-key"

# Optional: Use MQTT instead of HTTP
export ROBOMETRICS_USE_MQTT="true"
export ROBOMETRICS_MQTT_HOST="mqtt.robometrics.xyz"
export ROBOMETRICS_MQTT_PORT="1883"
```

### 2. Launch File Parameters

```bash
# ROS1
roslaunch robometrics_bridge robometrics_bridge.launch robot_id:=your-robot-id api_key:=your-api-key

# ROS2
ros2 launch robometrics_bridge robometrics_bridge.launch.py robot_id:=your-robot-id api_key:=your-api-key

# With MQTT enabled
roslaunch robometrics_bridge robometrics_bridge.launch robot_id:=your-robot-id api_key:=your-api-key use_mqtt:=true
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

## Custom Message Publishing

You can publish your own telemetry data directly using the `RobometricsMessage` format:

```python
#!/usr/bin/env python3
import rospy
from robometrics_bridge.msg import RobometricsMessage

def publish_telemetry():
    rospy.init_node('telemetry_publisher', anonymous=True)
    pub = rospy.Publisher('/robometrics/send', RobometricsMessage, queue_size=10)
    
    msg = RobometricsMessage()
    msg.robot_id = "my-robot-id"  # Will override bridge configuration
    msg.battery_level = 85
    msg.temperature = 32.5
    msg.status = "OK"
    msg.latitude = 37.7749
    msg.longitude = -122.4194
    
    pub.publish(msg)
    rospy.loginfo("Custom telemetry message published")
    
if __name__ == '__main__':
    try:
        publish_telemetry()
    except rospy.ROSInterruptException:
        pass
```

When a custom message is received, it is sent immediately to RoboMonitor.

## MQTT Support

The bridge supports MQTT for more reliable communication:

```bash
# Enable MQTT
roslaunch robometrics_bridge robometrics_bridge.launch use_mqtt:=true mqtt_host:=mqtt.robometrics.xyz mqtt_port:=1883
```

MQTT parameters:
- `use_mqtt` - Enable MQTT (default: false)
- `mqtt_host` - MQTT broker host (default: mqtt.robometrics.xyz)
- `mqtt_port` - MQTT broker port (default: 1883)
- `mqtt_topic` - Topic to publish to (default: robometrics/telemetry/{robot_id})

If MQTT connection fails, the bridge will automatically fall back to HTTP.

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
4. If using MQTT, ensure the broker is accessible

## License

MIT License
