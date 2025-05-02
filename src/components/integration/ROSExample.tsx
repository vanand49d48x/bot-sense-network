
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { useState } from "react";
import { toast } from "@/components/ui/sonner";
import { ClipboardCopy, Download } from "lucide-react";

export function ROSExample() {
  const [activeTab, setActiveTab] = useState("ros1");

  const ros1Example = `
#!/usr/bin/env python3

import rospy
import requests
import json
import math
from std_msgs.msg import Float32, String
from sensor_msgs.msg import BatteryState, Temperature, NavSatFix
from diagnostic_msgs.msg import DiagnosticStatus

class RoboMetricsBridge:
    def __init__(self):
        rospy.init_node('robometrics_bridge')
        
        # Configuration
        self.robot_id = rospy.get_param('~robot_id', 'YOUR_ROBOT_ID')
        self.api_key = rospy.get_param('~api_key', 'YOUR_API_KEY')
        self.api_endpoint = "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry"
        self.update_rate = rospy.get_param('~update_rate', 60.0)  # seconds
        
        # Robot state
        self.battery_level = 100.0
        self.temperature = 25.0
        self.status = "OK"
        self.location = {"latitude": 0.0, "longitude": 0.0}
        self.has_location = False
        
        # Subscribers
        rospy.Subscriber('/battery/state', BatteryState, self.battery_callback)
        rospy.Subscriber('/battery/percentage', Float32, self.battery_percentage_callback)
        rospy.Subscriber('/temperature', Temperature, self.temperature_callback)
        rospy.Subscriber('/diagnostics', DiagnosticStatus, self.diagnostics_callback)
        rospy.Subscriber('/fix', NavSatFix, self.location_callback)
        
        # Timer for sending data
        rospy.Timer(rospy.Duration(self.update_rate), self.send_data)
        
        rospy.loginfo("RoboMetrics bridge initialized")

    def battery_callback(self, msg):
        # Convert battery state to percentage
        self.battery_level = msg.percentage * 100.0
        if math.isnan(self.battery_level):
            # If percentage is not available, calculate from voltage
            if msg.voltage > 0:
                # Very simple linear mapping, adjust based on your battery
                self.battery_level = (msg.voltage - 11.0) / (12.6 - 11.0) * 100.0
                self.battery_level = max(0, min(100, self.battery_level))

    def battery_percentage_callback(self, msg):
        # Direct percentage input
        self.battery_level = msg.data

    def temperature_callback(self, msg):
        # Temperature in Celsius
        self.temperature = msg.temperature

    def diagnostics_callback(self, msg):
        # Map ROS diagnostic status to RoboMetrics status
        if msg.level == DiagnosticStatus.OK:
            self.status = "OK"
        elif msg.level == DiagnosticStatus.WARN:
            self.status = "WARNING"
        elif msg.level == DiagnosticStatus.ERROR:
            self.status = "ERROR"
        else:
            self.status = "OK"

    def location_callback(self, msg):
        # GPS location
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            self.location["latitude"] = msg.latitude
            self.location["longitude"] = msg.longitude
            self.has_location = True

    def send_data(self, event=None):
        try:
            # Prepare telemetry data
            payload = {
                "robotId": self.robot_id,
                "batteryLevel": self.battery_level,
                "temperature": self.temperature,
                "status": self.status
            }
            
            if self.has_location:
                payload["location"] = self.location
                
            # Send data to RoboMetrics
            headers = {
                "Content-Type": "application/json",
                "api-key": self.api_key
            }
            
            rospy.loginfo("Sending telemetry data to RoboMetrics")
            response = requests.post(
                self.api_endpoint, 
                headers=headers,
                data=json.dumps(payload)
            )
            
            if response.status_code == 200:
                rospy.loginfo("Data sent successfully")
            else:
                rospy.logwarn(f"Failed to send data. Status code: {response.status_code}")
                rospy.logwarn(f"Response: {response.text}")
                
        except Exception as e:
            rospy.logerr(f"Error sending data: {e}")

if __name__ == "__main__":
    try:
        bridge = RoboMetricsBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass`;

  const ros2Example = `
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import requests
import json
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import BatteryState, Temperature, NavSatFix
from diagnostic_msgs.msg import DiagnosticStatus

class RoboMetricsBridge(Node):
    def __init__(self):
        super().__init__('robometrics_bridge')
        
        # Configuration
        self.declare_parameter('robot_id', 'YOUR_ROBOT_ID')
        self.declare_parameter('api_key', 'YOUR_API_KEY')
        self.declare_parameter('update_rate', 60.0)
        
        self.robot_id = self.get_parameter('robot_id').value
        self.api_key = self.get_parameter('api_key').value
        self.api_endpoint = "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry"
        self.update_rate = self.get_parameter('update_rate').value  # seconds
        
        # Robot state
        self.battery_level = 100.0
        self.temperature = 25.0
        self.status = "OK"
        self.location = {"latitude": 0.0, "longitude": 0.0}
        self.has_location = False
        
        # Subscribers
        self.create_subscription(
            BatteryState, 
            '/battery/state', 
            self.battery_callback, 
            10
        )
        self.create_subscription(
            Float32, 
            '/battery/percentage', 
            self.battery_percentage_callback, 
            10
        )
        self.create_subscription(
            Temperature, 
            '/temperature', 
            self.temperature_callback, 
            10
        )
        self.create_subscription(
            DiagnosticStatus, 
            '/diagnostics', 
            self.diagnostics_callback, 
            10
        )
        self.create_subscription(
            NavSatFix, 
            '/fix', 
            self.location_callback, 
            10
        )
        
        # Timer for sending data
        self.timer = self.create_timer(self.update_rate, self.send_data)
        
        self.get_logger().info("RoboMetrics bridge initialized")

    def battery_callback(self, msg):
        # Convert battery state to percentage
        self.battery_level = msg.percentage * 100.0
        if math.isnan(self.battery_level):
            # If percentage is not available, calculate from voltage
            if msg.voltage > 0:
                # Very simple linear mapping, adjust based on your battery
                self.battery_level = (msg.voltage - 11.0) / (12.6 - 11.0) * 100.0
                self.battery_level = max(0, min(100, self.battery_level))

    def battery_percentage_callback(self, msg):
        # Direct percentage input
        self.battery_level = msg.data

    def temperature_callback(self, msg):
        # Temperature in Celsius
        self.temperature = msg.temperature

    def diagnostics_callback(self, msg):
        # Map ROS diagnostic status to RoboMetrics status
        if msg.level == DiagnosticStatus.OK:
            self.status = "OK"
        elif msg.level == DiagnosticStatus.WARN:
            self.status = "WARNING"
        elif msg.level == DiagnosticStatus.ERROR:
            self.status = "ERROR"
        else:
            self.status = "OK"

    def location_callback(self, msg):
        # GPS location
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            self.location["latitude"] = msg.latitude
            self.location["longitude"] = msg.longitude
            self.has_location = True

    def send_data(self):
        try:
            # Prepare telemetry data
            payload = {
                "robotId": self.robot_id,
                "batteryLevel": self.battery_level,
                "temperature": self.temperature,
                "status": self.status
            }
            
            if self.has_location:
                payload["location"] = self.location
                
            # Send data to RoboMetrics
            headers = {
                "Content-Type": "application/json",
                "api-key": self.api_key
            }
            
            self.get_logger().info("Sending telemetry data to RoboMetrics")
            response = requests.post(
                self.api_endpoint, 
                headers=headers,
                data=json.dumps(payload)
            )
            
            if response.status_code == 200:
                self.get_logger().info("Data sent successfully")
            else:
                self.get_logger().warn(f"Failed to send data. Status code: {response.status_code}")
                self.get_logger().warn(f"Response: {response.text}")
                
        except Exception as e:
            self.get_logger().error(f"Error sending data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RoboMetricsBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()`;

  const launchFileRos1 = `
<launch>
  <node name="robometrics_bridge" pkg="robometrics_bridge" type="robometrics_bridge.py" output="screen">
    <param name="robot_id" value="YOUR_ROBOT_ID" />
    <param name="api_key" value="YOUR_API_KEY" />
    <param name="update_rate" value="60.0" />
  </node>
</launch>`;

  const launchFileRos2 = `
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='YOUR_ROBOT_ID',
        description='Robot ID for RoboMetrics'
    )
    
    api_key_arg = DeclareLaunchArgument(
        'api_key',
        default_value='YOUR_API_KEY',
        description='API Key for RoboMetrics'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='60.0',
        description='Update rate in seconds for sending data'
    )
    
    # Create the node
    robometrics_node = Node(
        package='robometrics_bridge',
        executable='robometrics_bridge',
        name='robometrics_bridge',
        parameters=[{
            'robot_id': LaunchConfiguration('robot_id'),
            'api_key': LaunchConfiguration('api_key'),
            'update_rate': LaunchConfiguration('update_rate')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        robot_id_arg,
        api_key_arg,
        update_rate_arg,
        robometrics_node
    ])`;

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
          Integrate your ROS-powered robots with RoboMetrics using these bridge nodes. These examples show how to subscribe to standard ROS topics and send the data to the platform.
        </CardDescription>
      </CardHeader>
      <CardContent>
        <Tabs defaultValue="ros1" value={activeTab} onValueChange={setActiveTab}>
          <TabsList className="mb-4">
            <TabsTrigger value="ros1">ROS1 Python</TabsTrigger>
            <TabsTrigger value="ros2">ROS2 Python</TabsTrigger>
            <TabsTrigger value="launch">Launch Files</TabsTrigger>
          </TabsList>
          <TabsContent value="ros1" className="relative">
            <div className="flex justify-end gap-2 mb-2">
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => copyToClipboard(ros1Example, "ROS1")}
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
                onClick={() => copyToClipboard(ros2Example, "ROS2")}
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
          <TabsContent value="launch" className="relative space-y-6">
            <div>
              <div className="flex justify-between items-center mb-2">
                <h3 className="text-sm font-semibold">ROS1 Launch File</h3>
                <div className="flex gap-2">
                  <Button 
                    variant="outline" 
                    size="sm"
                    onClick={() => copyToClipboard(launchFileRos1, "ROS1 Launch")}
                  >
                    <ClipboardCopy className="h-4 w-4 mr-1" /> Copy
                  </Button>
                  <Button 
                    variant="outline" 
                    size="sm"
                    onClick={() => downloadFile(launchFileRos1, "robometrics.launch", "text/plain")}
                  >
                    <Download className="h-4 w-4 mr-1" /> Download
                  </Button>
                </div>
              </div>
              <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
                {launchFileRos1}
              </pre>
            </div>
            <div>
              <div className="flex justify-between items-center mb-2">
                <h3 className="text-sm font-semibold">ROS2 Launch File</h3>
                <div className="flex gap-2">
                  <Button 
                    variant="outline" 
                    size="sm"
                    onClick={() => copyToClipboard(launchFileRos2, "ROS2 Launch")}
                  >
                    <ClipboardCopy className="h-4 w-4 mr-1" /> Copy
                  </Button>
                  <Button 
                    variant="outline" 
                    size="sm"
                    onClick={() => downloadFile(launchFileRos2, "robometrics_launch.py", "text/plain")}
                  >
                    <Download className="h-4 w-4 mr-1" /> Download
                  </Button>
                </div>
              </div>
              <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
                {launchFileRos2}
              </pre>
            </div>
          </TabsContent>
        </Tabs>
      </CardContent>
    </Card>
  );
}
