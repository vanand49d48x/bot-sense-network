
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { useState } from "react";
import { toast } from "@/components/ui/sonner";
import { ClipboardCopy, Download } from "lucide-react";

export function ROSExample() {
  const [activeTab, setActiveTab] = useState("ros1");

  const ros1Example = `#!/usr/bin/env python3
import rospy
import requests
import json
from std_msgs.msg import Float32, Int32, String
from sensor_msgs.msg import BatteryState, Temperature, NavSatFix
from geometry_msgs.msg import Point

# RoboMetrics configuration
API_ENDPOINT = "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry"
ROBOT_ID = "YOUR_ROBOT_ID"
API_KEY = "YOUR_ROBOT_API_KEY"

# Global variables to store the latest sensor values
battery_level = 100
temperature = 25.0
latitude = 0.0
longitude = 0.0
robot_status = "OK"
# Custom telemetry variables
motor_speed = 0
error_count = 0
arm_position = "retracted"

# Callback functions for ROS topics
def battery_callback(data):
    global battery_level
    battery_level = data.percentage * 100

def temperature_callback(data):
    global temperature
    temperature = data.temperature

def location_callback(data):
    global latitude, longitude
    latitude = data.latitude
    longitude = data.longitude

def status_callback(data):
    global robot_status
    robot_status = data.data

# Custom telemetry callbacks
def motor_speed_callback(data):
    global motor_speed
    motor_speed = data.data

def error_count_callback(data):
    global error_count
    error_count = data.data

def arm_position_callback(data):
    global arm_position
    arm_position = data.data

def send_telemetry():
    # Prepare telemetry data
    data = {
        "robotId": ROBOT_ID,
        "batteryLevel": battery_level,
        "temperature": temperature,
        "status": robot_status,
        "location": {
            "latitude": latitude,
            "longitude": longitude
        },
        "customTelemetry": {
            "motorSpeed": motor_speed,
            "errorCount": error_count,
            "armPosition": arm_position
        }
    }
    
    # Send HTTP request
    headers = {
        "Content-Type": "application/json",
        "api-key": API_KEY
    }
    
    try:
        response = requests.post(
            API_ENDPOINT,
            headers=headers,
            data=json.dumps(data)
        )
        rospy.loginfo("Telemetry sent, status: %d", response.status_code)
    except Exception as e:
        rospy.logerr("Error sending telemetry: %s", str(e))

def main():
    # Initialize ROS node
    rospy.init_node('robometrics_bridge', anonymous=True)
    
    # Subscribe to standard topics
    rospy.Subscriber('battery', BatteryState, battery_callback)
    rospy.Subscriber('temperature', Temperature, temperature_callback)
    rospy.Subscriber('gps', NavSatFix, location_callback)
    rospy.Subscriber('status', String, status_callback)
    
    # Subscribe to custom telemetry topics
    rospy.Subscriber('motor_speed', Int32, motor_speed_callback)
    rospy.Subscriber('error_count', Int32, error_count_callback)
    rospy.Subscriber('arm_position', String, arm_position_callback)
    
    # Send telemetry at regular intervals
    rate = rospy.Rate(1/60)  # Send every 60 seconds
    
    while not rospy.is_shutdown():
        send_telemetry()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass`;

  const ros2Example = `#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import requests
import json
import time
from sensor_msgs.msg import BatteryState, Temperature, NavSatFix
from std_msgs.msg import Float32, Int32, String

# RoboMetrics configuration
API_ENDPOINT = "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry"
ROBOT_ID = "YOUR_ROBOT_ID"
API_KEY = "YOUR_ROBOT_API_KEY"

class RoboMetricsNode(Node):
    def __init__(self):
        super().__init__('robometrics_bridge')
        
        # Initialize sensor values
        self.battery_level = 100
        self.temperature = 25.0
        self.latitude = 0.0
        self.longitude = 0.0
        self.robot_status = "OK"
        # Custom telemetry variables
        self.motor_speed = 0
        self.error_count = 0
        self.arm_position = "retracted"
        
        # Create subscribers for standard topics
        self.battery_sub = self.create_subscription(
            BatteryState, 
            'battery',
            self.battery_callback,
            10)
            
        self.temp_sub = self.create_subscription(
            Temperature, 
            'temperature',
            self.temperature_callback,
            10)
            
        self.gps_sub = self.create_subscription(
            NavSatFix, 
            'gps',
            self.location_callback,
            10)
            
        self.status_sub = self.create_subscription(
            String, 
            'status',
            self.status_callback,
            10)
        
        # Create subscribers for custom telemetry topics
        self.motor_speed_sub = self.create_subscription(
            Int32, 
            'motor_speed',
            self.motor_speed_callback,
            10)
            
        self.error_count_sub = self.create_subscription(
            Int32, 
            'error_count',
            self.error_count_callback,
            10)
            
        self.arm_position_sub = self.create_subscription(
            String, 
            'arm_position',
            self.arm_position_callback,
            10)
        
        # Create timer for sending telemetry
        self.timer = self.create_timer(60.0, self.send_telemetry)
        self.get_logger().info('RoboMetrics bridge started')
    
    # Callback functions
    def battery_callback(self, msg):
        self.battery_level = msg.percentage * 100
    
    def temperature_callback(self, msg):
        self.temperature = msg.temperature
    
    def location_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
    
    def status_callback(self, msg):
        self.robot_status = msg.data
    
    # Custom telemetry callbacks
    def motor_speed_callback(self, msg):
        self.motor_speed = msg.data
    
    def error_count_callback(self, msg):
        self.error_count = msg.data
    
    def arm_position_callback(self, msg):
        self.arm_position = msg.data
    
    # Send telemetry to RoboMetrics
    def send_telemetry(self):
        # Prepare telemetry data
        data = {
            "robotId": ROBOT_ID,
            "batteryLevel": self.battery_level,
            "temperature": self.temperature,
            "status": self.robot_status,
            "location": {
                "latitude": self.latitude,
                "longitude": self.longitude
            },
            "customTelemetry": {
                "motorSpeed": self.motor_speed,
                "errorCount": self.error_count,
                "armPosition": self.arm_position
            }
        }
        
        # Send HTTP request
        headers = {
            "Content-Type": "application/json",
            "api-key": API_KEY
        }
        
        try:
            response = requests.post(
                API_ENDPOINT,
                headers=headers,
                data=json.dumps(data)
            )
            self.get_logger().info(f'Telemetry sent, status: {response.status_code}')
        except Exception as e:
            self.get_logger().error(f'Error sending telemetry: {str(e)}')

def main():
    rclpy.init()
    node = RoboMetricsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  const cppExample = `#include <ros/ros.h>
#include <curl/curl.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <string>
#include <sstream>

// RoboMetrics configuration
const std::string API_ENDPOINT = "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry";
const std::string ROBOT_ID = "YOUR_ROBOT_ID";
const std::string API_KEY = "YOUR_ROBOT_API_KEY";

// Global variables to store sensor values
float battery_level = 100.0;
float temperature = 25.0;
double latitude = 0.0;
double longitude = 0.0;
std::string robot_status = "OK";
// Custom telemetry variables
int motor_speed = 0;
int error_count = 0;
std::string arm_position = "retracted";

// Callback functions
void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
    battery_level = msg->percentage * 100.0f;
}

void temperatureCallback(const sensor_msgs::Temperature::ConstPtr& msg) {
    temperature = msg->temperature;
}

void locationCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    latitude = msg->latitude;
    longitude = msg->longitude;
}

void statusCallback(const std_msgs::String::ConstPtr& msg) {
    robot_status = msg->data;
}

// Custom telemetry callbacks
void motorSpeedCallback(const std_msgs::Int32::ConstPtr& msg) {
    motor_speed = msg->data;
}

void errorCountCallback(const std_msgs::Int32::ConstPtr& msg) {
    error_count = msg->data;
}

void armPositionCallback(const std_msgs::String::ConstPtr& msg) {
    arm_position = msg->data;
}

// Helper function for CURL
size_t writeCallback(char* ptr, size_t size, size_t nmemb, std::string* data) {
    data->append(ptr, size * nmemb);
    return size * nmemb;
}

void sendTelemetry() {
    CURL* curl;
    CURLcode res;
    std::string readBuffer;
    
    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();
    
    if(curl) {
        // Prepare JSON payload
        std::stringstream json_data;
        json_data << "{"
                  << "\\\"robotId\\\":\\\"" << ROBOT_ID << "\\\","
                  << "\\\"batteryLevel\\\":" << battery_level << ","
                  << "\\\"temperature\\\":" << temperature << ","
                  << "\\\"status\\\":\\\"" << robot_status << "\\\","
                  << "\\\"location\\\":{"
                  << "\\\"latitude\\\":" << latitude << ","
                  << "\\\"longitude\\\":" << longitude
                  << "},"
                  << "\\\"customTelemetry\\\":{"
                  << "\\\"motorSpeed\\\":" << motor_speed << ","
                  << "\\\"errorCount\\\":" << error_count << ","
                  << "\\\"armPosition\\\":\\\"" << arm_position << "\\\""
                  << "}"
                  << "}";
                  
        std::string payload = json_data.str();
        
        // Set up CURL request
        curl_easy_setopt(curl, CURLOPT_URL, API_ENDPOINT.c_str());
        
        // Set headers
        struct curl_slist* headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        std::string auth_header = "api-key: " + API_KEY;
        headers = curl_slist_append(headers, auth_header.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        
        // Set request body
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
        
        // Set callback function
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        
        // Perform request
        res = curl_easy_perform(curl);
        
        // Check for errors
        if(res != CURLE_OK) {
            ROS_ERROR("Failed to send telemetry: %s", curl_easy_strerror(res));
        } else {
            long response_code;
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
            ROS_INFO("Telemetry sent, status: %ld", response_code);
        }
        
        // Clean up
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
    }
    
    curl_global_cleanup();
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "robometrics_bridge");
    ros::NodeHandle nh;
    
    // Subscribe to topics
    ros::Subscriber battery_sub = nh.subscribe("battery", 10, batteryCallback);
    ros::Subscriber temp_sub = nh.subscribe("temperature", 10, temperatureCallback);
    ros::Subscriber gps_sub = nh.subscribe("gps", 10, locationCallback);
    ros::Subscriber status_sub = nh.subscribe("status", 10, statusCallback);
    
    // Subscribe to custom telemetry topics
    ros::Subscriber motor_speed_sub = nh.subscribe("motor_speed", 10, motorSpeedCallback);
    ros::Subscriber error_count_sub = nh.subscribe("error_count", 10, errorCountCallback);
    ros::Subscriber arm_position_sub = nh.subscribe("arm_position", 10, armPositionCallback);
    
    // Set up timer
    ros::Rate rate(1.0/60.0); // 60 seconds interval
    
    while (ros::ok()) {
        // Process callbacks
        ros::spinOnce();
        
        // Send telemetry
        sendTelemetry();
        
        // Wait for next iteration
        rate.sleep();
    }
    
    return 0;
}`;

  const copyToClipboard = (text: string, type: string) => {
    navigator.clipboard.writeText(text);
    toast.success(`${type} code copied to clipboard`);
  };

  const downloadFile = (content: string, fileName: string, fileType: string) => {
    const blob = new Blob([content], { type: fileType });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = fileName;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    toast.success(`${fileName} downloaded`);
  };

  return (
    <Card>
      <CardHeader>
        <CardTitle>ROS Integration</CardTitle>
        <CardDescription>
          Connect ROS-based robots to RoboMetrics with these examples. You can bridge your existing ROS topics to send telemetry data to the platform.
        </CardDescription>
      </CardHeader>
      <CardContent>
        <Tabs defaultValue="ros1" value={activeTab} onValueChange={setActiveTab}>
          <TabsList className="mb-4">
            <TabsTrigger value="ros1">ROS 1 (Python)</TabsTrigger>
            <TabsTrigger value="ros2">ROS 2 (Python)</TabsTrigger>
            <TabsTrigger value="cpp">ROS 1 (C++)</TabsTrigger>
          </TabsList>
          <TabsContent value="ros1" className="relative">
            <div className="flex justify-end gap-2 mb-2">
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => copyToClipboard(ros1Example, "ROS 1 Python")}
              >
                <ClipboardCopy className="h-4 w-4 mr-1" /> Copy Code
              </Button>
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => downloadFile(ros1Example, "robometrics_bridge.py", "text/plain")}
              >
                <Download className="h-4 w-4 mr-1" /> Download
              </Button>
            </div>
            <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
              {ros1Example}
            </pre>
          </TabsContent>
          <TabsContent value="ros2" className="relative">
            <div className="flex justify-end gap-2 mb-2">
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => copyToClipboard(ros2Example, "ROS 2 Python")}
              >
                <ClipboardCopy className="h-4 w-4 mr-1" /> Copy Code
              </Button>
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => downloadFile(ros2Example, "robometrics_bridge_ros2.py", "text/plain")}
              >
                <Download className="h-4 w-4 mr-1" /> Download
              </Button>
            </div>
            <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
              {ros2Example}
            </pre>
          </TabsContent>
          <TabsContent value="cpp" className="relative">
            <div className="flex justify-end gap-2 mb-2">
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => copyToClipboard(cppExample, "ROS C++")}
              >
                <ClipboardCopy className="h-4 w-4 mr-1" /> Copy Code
              </Button>
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => downloadFile(cppExample, "robometrics_bridge.cpp", "text/plain")}
              >
                <Download className="h-4 w-4 mr-1" /> Download
              </Button>
            </div>
            <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
              {cppExample}
            </pre>
          </TabsContent>
        </Tabs>
        <div className="mt-6 bg-muted-foreground/10 p-4 rounded-md">
          <h3 className="text-md font-semibold mb-2">Custom Telemetry Tips for ROS</h3>
          <ul className="list-disc pl-6 space-y-1 text-sm">
            <li>Create dedicated ROS topics for your custom telemetry values</li>
            <li>For structured data, use custom message types</li>
            <li>The examples subscribe to <code>motor_speed</code>, <code>error_count</code>, and <code>arm_position</code> topics</li>
            <li>Adapt the example code to include your own specific topics and data types</li>
          </ul>
        </div>
      </CardContent>
    </Card>
  );
}
