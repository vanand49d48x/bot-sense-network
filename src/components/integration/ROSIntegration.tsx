
import React from 'react';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Button } from "@/components/ui/button";
import { ClipboardCopy, Download } from "lucide-react";
import { toast } from "@/components/ui/sonner";
import { Badge } from "@/components/ui/badge";
import { Separator } from "@/components/ui/separator";

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

  const launchCommand = `# ROS1
roslaunch robometrics_bridge robometrics_bridge.launch robot_id:=your-robot-id api_key:=your-api-key

# ROS2
ros2 launch robometrics_bridge robometrics_bridge.launch.py robot_id:=your-robot-id api_key:=your-api-key`;

  const mqttLaunchCommand = `# ROS1 with MQTT
roslaunch robometrics_bridge robometrics_bridge.launch robot_id:=your-robot-id api_key:=your-api-key use_mqtt:=true mqtt_host:=mqtt.robometrics.xyz

# ROS2 with MQTT
ros2 launch robometrics_bridge robometrics_bridge.launch.py robot_id:=your-robot-id api_key:=your-api-key use_mqtt:=true mqtt_host:=mqtt.robometrics.xyz`;

  const customMessageExample = `#!/usr/bin/env python3
import rospy
from robometrics_bridge.msg import RobometricsMessage

def publish_telemetry():
    # Initialize node
    rospy.init_node('telemetry_publisher', anonymous=True)
    
    # Create publisher for RobometricsMessage
    pub = rospy.Publisher('/robometrics/send', RobometricsMessage, queue_size=10)
    
    # Create message
    msg = RobometricsMessage()
    msg.robot_id = "my-robot-id"  # Will override the bridge configuration
    msg.battery_level = 85
    msg.temperature = 32.5
    msg.status = "OK"
    msg.latitude = 37.7749
    msg.longitude = -122.4194
    
    # Publish message (will be sent immediately by the bridge)
    pub.publish(msg)
    rospy.loginfo("Custom telemetry message published")
    
if __name__ == '__main__':
    try:
        publish_telemetry()
    except rospy.ROSInterruptException:
        pass`;

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
            <TabsTrigger value="mqtt">MQTT</TabsTrigger>
            <TabsTrigger value="custom">Custom Message</TabsTrigger>
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
              
              <Badge variant="outline" className="mb-2">ROS1 & ROS2 Compatible</Badge>
              
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
          
          <TabsContent value="mqtt">
            <div className="space-y-4">
              <div className="flex items-center gap-2 mb-2">
                <h3 className="text-lg font-semibold">MQTT Connection</h3>
                <Badge>Recommended</Badge>
              </div>
              
              <p className="text-sm text-muted-foreground">
                MQTT provides a more efficient and reliable connection for sending telemetry data, especially in environments with unstable internet connections.
              </p>
              
              <div className="p-4 bg-muted rounded-md relative">
                <Button 
                  variant="ghost" 
                  size="icon"
                  className="absolute top-2 right-2"
                  onClick={() => copyToClipboard(mqttLaunchCommand)}
                >
                  <ClipboardCopy size={16} />
                </Button>
                <pre className="text-xs overflow-x-auto whitespace-pre-wrap">
                  {mqttLaunchCommand}
                </pre>
              </div>
              
              <div className="mt-4">
                <h3 className="text-lg font-semibold mb-2">MQTT Configuration</h3>
                <ul className="list-disc pl-5 space-y-2 text-sm">
                  <li><code className="bg-background px-1.5 py-0.5 rounded">use_mqtt</code> - Enable MQTT (default: false)</li>
                  <li><code className="bg-background px-1.5 py-0.5 rounded">mqtt_host</code> - MQTT broker host (default: mqtt.robometrics.xyz)</li>
                  <li><code className="bg-background px-1.5 py-0.5 rounded">mqtt_port</code> - MQTT broker port (default: 1883)</li>
                  <li><code className="bg-background px-1.5 py-0.5 rounded">mqtt_topic</code> - Topic to publish to (default: robometrics/telemetry/{robot_id})</li>
                </ul>
              </div>
              
              <p className="text-sm text-muted-foreground mt-4">
                You can also set these parameters using environment variables:
                <code className="block bg-background p-2 rounded mt-2 text-xs">
                  export ROBOMETRICS_USE_MQTT=true<br/>
                  export ROBOMETRICS_MQTT_HOST=mqtt.robometrics.xyz<br/>
                  export ROBOMETRICS_MQTT_PORT=1883
                </code>
              </p>
            </div>
          </TabsContent>
          
          <TabsContent value="custom">
            <div className="space-y-4">
              <div className="flex items-center gap-2 mb-2">
                <h3 className="text-lg font-semibold">Custom Message Publishing</h3>
                <Badge variant="secondary">Advanced</Badge>
              </div>
              
              <p className="text-sm text-muted-foreground">
                You can publish your own telemetry data directly using the RobometricsMessage format.
                Just send a message to the <code className="bg-background px-1.5 py-0.5 rounded">/robometrics/send</code> topic.
              </p>
              
              <div className="p-4 bg-muted rounded-md relative">
                <Button 
                  variant="ghost" 
                  size="icon"
                  className="absolute top-2 right-2"
                  onClick={() => copyToClipboard(customMessageExample)}
                >
                  <ClipboardCopy size={16} />
                </Button>
                <pre className="text-xs overflow-x-auto whitespace-pre-wrap">
                  {customMessageExample}
                </pre>
              </div>
              
              <div className="mt-4">
                <h3 className="text-lg font-semibold mb-2">Message Fields</h3>
                <ul className="list-disc pl-5 space-y-2 text-sm">
                  <li><code className="bg-background px-1.5 py-0.5 rounded">robot_id</code> - Robot ID (overrides bridge config if provided)</li>
                  <li><code className="bg-background px-1.5 py-0.5 rounded">battery_level</code> - Battery percentage (0-100)</li>
                  <li><code className="bg-background px-1.5 py-0.5 rounded">temperature</code> - Temperature in Celsius</li>
                  <li><code className="bg-background px-1.5 py-0.5 rounded">status</code> - Status string ("OK", "WARNING", "ERROR")</li>
                  <li><code className="bg-background px-1.5 py-0.5 rounded">latitude</code> - Latitude coordinate</li>
                  <li><code className="bg-background px-1.5 py-0.5 rounded">longitude</code> - Longitude coordinate</li>
                  <li><code className="bg-background px-1.5 py-0.5 rounded">timestamp</code> - Optional timestamp</li>
                </ul>
              </div>
              
              <p className="text-sm font-medium mt-4">
                When a custom message is received, it is sent immediately to RoboMonitor, bypassing the regular update interval.
              </p>
            </div>
          </TabsContent>
        </Tabs>
      </CardContent>
    </Card>
  );
}
