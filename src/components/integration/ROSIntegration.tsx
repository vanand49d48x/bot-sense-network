
import React from 'react';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Button } from "@/components/ui/button";
import { ClipboardCopy, Download } from "lucide-react";
import { toast } from "@/components/ui/sonner";

export function ROSIntegration() {
  const copyToClipboard = (text: string) => {
    navigator.clipboard.writeText(text);
    toast("Code copied", {
      description: "The code has been copied to your clipboard"
    });
  };
  
  const rosSetupScript = `#!/bin/bash
# Setup script for RoboMonitor ROS Bridge

# Create workspace directory if needed
mkdir -p ~/robometrics_ws/src
cd ~/robometrics_ws/src

# Clone the repository
git clone https://github.com/robometrics/robometrics-ros-bridge robometrics_bridge

# Install Python dependencies
pip install -r robometrics_bridge/requirements.txt

# Build the workspace (for ROS1)
cd ..
if [ -n "$ROS_VERSION" ] && [ "$ROS_VERSION" -eq 2 ]; then
  colcon build --packages-select robometrics_bridge
  echo "ROS2 workspace built successfully!"
else
  catkin_make
  echo "ROS1 workspace built successfully!"
fi

# Source the workspace
if [ -n "$ROS_VERSION" ] && [ "$ROS_VERSION" -eq 2 ]; then
  echo "source ~/robometrics_ws/install/setup.bash" >> ~/.bashrc
else
  echo "source ~/robometrics_ws/devel/setup.bash" >> ~/.bashrc
fi

echo "RoboMonitor ROS bridge installed!"
echo "Configure your robot ID and API key with:"
echo "export ROBOMETRICS_ROBOT_ID=your-robot-id"
echo "export ROBOMETRICS_API_KEY=your-api-key"`;

  const rosNodeCode = `#!/usr/bin/env python3
import rospy
import requests
from sensor_msgs.msg import BatteryState, Temperature, NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class RoboMetricsBridge:
    def __init__(self):
        # Get parameters
        self.robot_id = rospy.get_param('~robot_id', 'robot-ros-001')
        self.api_key = rospy.get_param('~api_key', '')
        self.api_url = rospy.get_param('~api_url', 'https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry')
        
        # Set up telemetry structure
        self.telemetry = {
            "robotId": self.robot_id,
            "batteryLevel": 0,
            "temperature": 0,
            "status": "OK",
            "location": {"latitude": 0, "longitude": 0}
        }
        
        # Set up subscribers
        rospy.Subscriber('/battery_state', BatteryState, self.battery_callback)
        rospy.Subscriber('/temperature', Temperature, self.temp_callback)
        rospy.Subscriber('/robot_status', String, self.status_callback)
        
        # Try GPS first, fall back to odometry
        try:
            rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        except:
            rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
    def send_telemetry(self):
        # Send data to RoboMonitor platform
        response = requests.post(
            self.api_url,
            headers={"api-key": self.api_key, "Content-Type": "application/json"},
            json=self.telemetry
        )
        rospy.loginfo(f"Sent telemetry: {response.status_code}")
        
    def run(self):
        rate = rospy.Rate(0.1)  # every 10 seconds
        while not rospy.is_shutdown():
            self.send_telemetry()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('robometrics_bridge')
    bridge = RoboMetricsBridge()
    bridge.run()`;

  const pythonRequirements = `requests>=2.25.0`;

  const launchCommand = `# ROS1
roslaunch robometrics_bridge robometrics_bridge.launch robot_id:=your-robot-id api_key:=your-api-key

# ROS2
ros2 launch robometrics_bridge robometrics_bridge.launch.py robot_id:=your-robot-id api_key:=your-api-key`;

  return (
    <Card className="mt-6">
      <CardHeader>
        <CardTitle>ROS Integration</CardTitle>
        <CardDescription>
          Connect your ROS-based robots to RoboMonitor using our official ROS bridge
        </CardDescription>
      </CardHeader>
      <CardContent>
        <Tabs defaultValue="install">
          <TabsList className="mb-4">
            <TabsTrigger value="install">Installation</TabsTrigger>
            <TabsTrigger value="usage">Usage</TabsTrigger>
            <TabsTrigger value="code">Example Code</TabsTrigger>
          </TabsList>
          
          <TabsContent value="install">
            <div className="space-y-4">
              <div className="flex justify-between items-center">
                <h3 className="text-lg font-semibold">Quick Install</h3>
                <Button variant="outline" size="sm" className="flex gap-1" onClick={() => window.open('https://github.com/robometrics/robometrics-ros-bridge', '_blank')}>
                  <Download className="h-4 w-4" />
                  <span>Download</span>
                </Button>
              </div>
              
              <div className="p-4 bg-muted rounded-md relative">
                <Button 
                  variant="ghost" 
                  size="icon"
                  className="absolute top-2 right-2"
                  onClick={() => copyToClipboard(rosSetupScript)}
                >
                  <ClipboardCopy size={16} />
                </Button>
                <pre className="text-xs overflow-x-auto whitespace-pre-wrap">
                  {rosSetupScript}
                </pre>
              </div>
              
              <p className="text-sm text-muted-foreground mt-4">
                This script will set up a catkin/colcon workspace and install all necessary files.
                After running, set your robot ID and API key as shown in the final instructions.
              </p>
            </div>
          </TabsContent>
          
          <TabsContent value="usage">
            <div className="space-y-4">
              <div>
                <h3 className="text-lg font-semibold mb-2">Starting the Bridge</h3>
                <div className="p-4 bg-muted rounded-md relative">
                  <Button 
                    variant="ghost" 
                    size="icon"
                    className="absolute top-2 right-2"
                    onClick={() => copyToClipboard(launchCommand)}
                  >
                    <ClipboardCopy size={16} />
                  </Button>
                  <pre className="text-xs overflow-x-auto">
                    {launchCommand}
                  </pre>
                </div>
              </div>
              
              <div className="mt-4">
                <h3 className="text-lg font-semibold mb-2">Required Topics</h3>
                <ul className="list-disc pl-5 space-y-2 text-sm">
                  <li><code className="bg-background px-1.5 py-0.5 rounded">/battery_state</code> - Battery percentage</li>
                  <li><code className="bg-background px-1.5 py-0.5 rounded">/temperature</code> - System temperature</li>
                  <li><code className="bg-background px-1.5 py-0.5 rounded">/robot_status</code> - "OK", "WARNING" or "ERROR"</li>
                  <li><code className="bg-background px-1.5 py-0.5 rounded">/gps/fix</code> or <code className="bg-background px-1.5 py-0.5 rounded">/odom</code> - Location data</li>
                </ul>
              </div>
              
              <p className="text-sm text-muted-foreground mt-4">
                If your topics have different names, use ROS topic remapping:
                <code className="block bg-background p-2 rounded mt-2 text-xs">roslaunch robometrics_bridge robometrics_bridge.launch battery_state:=/my_robot/battery</code>
              </p>
            </div>
          </TabsContent>
          
          <TabsContent value="code">
            <Tabs defaultValue="node">
              <TabsList className="mb-4">
                <TabsTrigger value="node">ROS Node</TabsTrigger>
                <TabsTrigger value="requirements">Requirements</TabsTrigger>
              </TabsList>
              
              <TabsContent value="node" className="relative">
                <Button 
                  variant="ghost" 
                  size="icon"
                  className="absolute top-2 right-2"
                  onClick={() => copyToClipboard(rosNodeCode)}
                >
                  <ClipboardCopy size={16} />
                </Button>
                <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
                  {rosNodeCode}
                </pre>
              </TabsContent>
              
              <TabsContent value="requirements" className="relative">
                <Button 
                  variant="ghost" 
                  size="icon"
                  className="absolute top-2 right-2"
                  onClick={() => copyToClipboard(pythonRequirements)}
                >
                  <ClipboardCopy size={16} />
                </Button>
                <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
                  {pythonRequirements}
                </pre>
              </TabsContent>
            </Tabs>
          </TabsContent>
        </Tabs>
      </CardContent>
    </Card>
  );
}
