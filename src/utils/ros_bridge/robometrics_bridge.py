
#!/usr/bin/env python3
"""
ROS bridge for RoboMonitor platform.
Collects telemetry data from ROS topics and sends it via HTTP or MQTT.
"""

import os
import json
import time
import rospy
import requests
import paho.mqtt.client as mqtt
from sensor_msgs.msg import BatteryState, Temperature
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

# Import our custom message if available
try:
    from robometrics_bridge.msg import RobometricsMessage
except ImportError:
    rospy.logwarn("RobometricsMessage not found. Custom message topic will not be available.")
    RobometricsMessage = None

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
        
        # MQTT configuration
        self.use_mqtt = rospy.get_param('~use_mqtt', os.environ.get('ROBOMETRICS_USE_MQTT', 'false').lower() == 'true')
        self.mqtt_host = rospy.get_param('~mqtt_host', os.environ.get('ROBOMETRICS_MQTT_HOST', 'mqtt.robometrics.xyz'))
        self.mqtt_port = int(rospy.get_param('~mqtt_port', os.environ.get('ROBOMETRICS_MQTT_PORT', '1883')))
        self.mqtt_topic = rospy.get_param('~mqtt_topic', os.environ.get('ROBOMETRICS_MQTT_TOPIC', f'robometrics/telemetry/{self.robot_id}'))
        self.mqtt_client = None
        
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
        
        # Custom message subscriber
        if RobometricsMessage:
            rospy.Subscriber('/robometrics/send', RobometricsMessage, self.custom_message_callback)
            rospy.loginfo("Custom message topic /robometrics/send is available")
        
        # Initialize MQTT if needed
        if self.use_mqtt:
            self._init_mqtt()
            rospy.loginfo(f"RoboMetrics Bridge initialized for robot: {self.robot_id} (using MQTT)")
        else:
            rospy.loginfo(f"RoboMetrics Bridge initialized for robot: {self.robot_id} (using HTTP)")
        
        if not self.api_key:
            rospy.logwarn("API key not set. Please set ROBOMETRICS_API_KEY environment variable or ~api_key parameter")
    
    def _init_mqtt(self):
        """Initialize MQTT client connection."""
        try:
            client_id = f'robometrics-{self.robot_id}-{int(time.time())}'
            self.mqtt_client = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv5)
            self.mqtt_client.username_pw_set("robometrics", password=self.api_key)
            self.mqtt_client.on_connect = self._on_mqtt_connect
            self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
            
            self.mqtt_client.connect_async(self.mqtt_host, self.mqtt_port)
            self.mqtt_client.loop_start()
            
            rospy.loginfo(f"MQTT client initialized, connecting to {self.mqtt_host}:{self.mqtt_port}")
        except Exception as e:
            rospy.logerr(f"Failed to initialize MQTT: {e}")
            self.use_mqtt = False
    
    def _on_mqtt_connect(self, client, userdata, flags, rc, properties=None):
        """Callback when MQTT client connects."""
        if rc == 0:
            rospy.loginfo(f"Connected to MQTT broker: {self.mqtt_host}")
        else:
            rospy.logerr(f"Failed to connect to MQTT broker with code {rc}")
            self.use_mqtt = False
    
    def _on_mqtt_disconnect(self, client, userdata, rc):
        """Callback when MQTT client disconnects."""
        if rc != 0:
            rospy.logwarn(f"Unexpected MQTT disconnection with code {rc}. Reconnecting...")
            try:
                self.mqtt_client.reconnect()
            except Exception as e:
                rospy.logerr(f"Failed to reconnect to MQTT: {e}")
                self.use_mqtt = False
    
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
    
    def custom_message_callback(self, msg):
        """Handle custom RobometricsMessage."""
        # Override current telemetry with custom message data
        self.telemetry["robotId"] = msg.robot_id if msg.robot_id else self.telemetry["robotId"]
        self.telemetry["batteryLevel"] = msg.battery_level
        self.telemetry["temperature"] = msg.temperature
        self.telemetry["status"] = msg.status
        self.telemetry["location"]["latitude"] = msg.latitude
        self.telemetry["location"]["longitude"] = msg.longitude
        
        # Send immediately if a custom message is received
        self.send_telemetry()
    
    def send_telemetry(self):
        """Send telemetry data to RoboMonitor platform."""
        self.telemetry["timestamp"] = rospy.get_time()
        
        # Send via MQTT if enabled
        if self.use_mqtt and self.mqtt_client and self.mqtt_client.is_connected():
            try:
                result = self.mqtt_client.publish(
                    self.mqtt_topic,
                    json.dumps(self.telemetry),
                    qos=1  # At least once delivery
                )
                if result.rc == mqtt.MQTT_ERR_SUCCESS:
                    rospy.loginfo(f"Sent telemetry via MQTT: {self.mqtt_topic}")
                else:
                    rospy.logwarn(f"MQTT publish error: {result.rc}")
                    # Fall back to HTTP if MQTT fails
                    self._send_http()
                    
            except Exception as e:
                rospy.logerr(f"Failed to send telemetry via MQTT: {e}")
                # Fall back to HTTP
                self._send_http()
        else:
            # Use HTTP
            self._send_http()
    
    def _send_http(self):
        """Send telemetry using HTTP POST."""
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
            rospy.loginfo(f"Sent telemetry via HTTP (status: {status})")
            
            if status != 200:
                rospy.logwarn(f"Error sending telemetry: {response.text}")
                
        except Exception as e:
            rospy.logerr(f"Failed to send telemetry via HTTP: {e}")
    
    def run(self):
        """Main loop to regularly send telemetry data."""
        rate = rospy.Rate(self.update_rate)  # e.g. 0.1 Hz = every 10 seconds
        
        while not rospy.is_shutdown():
            if self.api_key:  # Only send if API key is configured
                self.send_telemetry()
            else:
                rospy.logwarn_throttle(60, "API key not set, skipping telemetry send")
            rate.sleep()
        
        # Cleanup
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

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
