
#!/usr/bin/env python3
"""
ROS bridge for RoboMonitor platform.
Collects telemetry data from ROS topics and sends it to RoboMonitor.
"""

import os
import json
import time
import rospy
import requests
from sensor_msgs.msg import BatteryState, Temperature
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

class RoboMetricsBridge:
    """Bridge between ROS and RoboMonitor platform."""
    
    def __init__(self):
        """Initialize the bridge with configuration parameters."""
        # Get parameters from ROS param server with defaults
        self.robot_id = rospy.get_param('~robot_id', os.environ.get('ROBOMETRICS_ROBOT_ID', 'robot-ros-001'))
        self.api_key = rospy.get_param('~api_key', os.environ.get('ROBOMETRICS_API_KEY', ''))
        self.api_url = rospy.get_param('~api_url', os.environ.get('ROBOMETRICS_API_URL', 
                                      'https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry'))
        self.update_rate = rospy.get_param('~update_rate', 0.1)  # 10 seconds default
        
        # Set up telemetry data structure
        self.telemetry = {
            "robotId": self.robot_id,
            "batteryLevel": 0,
            "temperature": 0,
            "status": "OK",
            "location": {"latitude": 0, "longitude": 0}
        }
        
        # Initialize subscribers
        rospy.Subscriber('/battery_state', BatteryState, self.battery_callback)
        rospy.Subscriber('/temperature', Temperature, self.temp_callback)
        rospy.Subscriber('/robot_status', String, self.status_callback)
        
        # Try to subscribe to either GPS or odometry
        self.using_gps = False
        try:
            rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
            self.using_gps = True
            rospy.loginfo("Using GPS data for location")
        except Exception:
            rospy.loginfo("GPS topic not available, falling back to odometry")
            rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        rospy.loginfo(f"RoboMetrics Bridge initialized for robot: {self.robot_id}")
        
        if not self.api_key:
            rospy.logwarn("API key not set. Please set ROBOMETRICS_API_KEY environment variable or ~api_key parameter")
    
    def battery_callback(self, msg):
        """Handle battery state messages."""
        # BatteryState percentage is between 0-1.0
        self.telemetry["batteryLevel"] = int(msg.percentage * 100)
        
    def temp_callback(self, msg):
        """Handle temperature messages."""
        self.telemetry["temperature"] = round(msg.temperature, 2)
    
    def status_callback(self, msg):
        """Handle robot status messages."""
        status = msg.data.upper()
        if status in ["ERROR", "WARNING", "OK"]:
            self.telemetry["status"] = status
    
    def gps_callback(self, msg):
        """Handle GPS position messages."""
        self.telemetry["location"]["latitude"] = msg.latitude
        self.telemetry["location"]["longitude"] = msg.longitude
    
    def odom_callback(self, msg):
        """Handle odometry messages (when GPS not available)."""
        # Using x,y coordinates from odometry as pseudo-location
        self.telemetry["location"]["latitude"] = msg.pose.pose.position.x
        self.telemetry["location"]["longitude"] = msg.pose.pose.position.y
    
    def send_telemetry(self):
        """Send telemetry data to RoboMonitor platform."""
        self.telemetry["timestamp"] = rospy.get_time()
        
        try:
            response = requests.post(
                self.api_url,
                headers={
                    "api-key": self.api_key,
                    "Content-Type": "application/json"
                },
                json=self.telemetry,
                timeout=5
            )
            status = response.status_code
            rospy.loginfo(f"Sent telemetry to RoboMonitor (status: {status})")
            
            if status != 200:
                rospy.logwarn(f"Error sending telemetry: {response.text}")
                
        except Exception as e:
            rospy.logerr(f"Failed to send telemetry: {e}")
    
    def run(self):
        """Main loop to regularly send telemetry data."""
        rate = rospy.Rate(self.update_rate)  # e.g. 0.1 Hz = every 10 seconds
        
        while not rospy.is_shutdown():
            if self.api_key:  # Only send if API key is configured
                self.send_telemetry()
            else:
                rospy.logwarn_throttle(60, "API key not set, skipping telemetry send")
            rate.sleep()

def main():
    """Entry point for the ROS node."""
    rospy.init_node('robometrics_bridge', anonymous=True)
    bridge = RoboMetricsBridge()
    
    try:
        bridge.run()
    except rospy.ROSInterruptException:
        pass
    
if __name__ == "__main__":
    main()
