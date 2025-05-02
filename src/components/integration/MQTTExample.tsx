
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { useState } from "react";
import { toast } from "@/components/ui/sonner";
import { ClipboardCopy, Download } from "lucide-react";

export function MQTTExample() {
  const [activeTab, setActiveTab] = useState("nodejs");

  const nodejsExample = `
const mqtt = require('mqtt');
const axios = require('axios');

// MQTT Configuration
const MQTT_BROKER = 'mqtt://broker.hivemq.com'; // Public broker for example, use your own in production
const TOPICS = {
  BATTERY: 'robometrics/YOUR_ROBOT_ID/battery',
  TEMPERATURE: 'robometrics/YOUR_ROBOT_ID/temperature',
  STATUS: 'robometrics/YOUR_ROBOT_ID/status',
  LOCATION: 'robometrics/YOUR_ROBOT_ID/location',
  ALL: 'robometrics/YOUR_ROBOT_ID/all' // Combined topic for all data
};

// RoboMetrics configuration
const ROBOMETRICS_ENDPOINT = 'https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry';
const ROBOT_ID = 'YOUR_ROBOT_ID';
const API_KEY = 'YOUR_API_KEY';

// State to hold latest robot telemetry
const robotState = {
  batteryLevel: 100,
  temperature: 25.0,
  status: 'OK',
  location: {
    latitude: 0,
    longitude: 0
  },
  hasLocationData: false
};

// Connect to MQTT broker
console.log(\`Connecting to MQTT broker: \${MQTT_BROKER}\`);
const client = mqtt.connect(MQTT_BROKER);

client.on('connect', () => {
  console.log('Connected to MQTT broker');
  
  // Subscribe to all relevant topics
  Object.values(TOPICS).forEach(topic => {
    client.subscribe(topic, (err) => {
      if (err) {
        console.error(\`Error subscribing to \${topic}:\`, err);
      } else {
        console.log(\`Subscribed to \${topic}\`);
      }
    });
  });
  
  // Setup periodic telemetry sending
  setInterval(sendTelemetry, 60000); // Send every 60 seconds
});

client.on('message', (topic, message) => {
  const payload = message.toString();
  console.log(\`Received message on \${topic}: \${payload}\`);
  
  try {
    // Process message based on topic
    if (topic === TOPICS.BATTERY) {
      robotState.batteryLevel = parseFloat(payload);
      console.log(\`Updated battery level: \${robotState.batteryLevel}%\`);
    }
    else if (topic === TOPICS.TEMPERATURE) {
      robotState.temperature = parseFloat(payload);
      console.log(\`Updated temperature: \${robotState.temperature}°C\`);
    }
    else if (topic === TOPICS.STATUS) {
      robotState.status = payload;
      console.log(\`Updated status: \${robotState.status}\`);
    }
    else if (topic === TOPICS.LOCATION) {
      try {
        const locationData = JSON.parse(payload);
        if (locationData.latitude !== undefined && locationData.longitude !== undefined) {
          robotState.location = {
            latitude: locationData.latitude,
            longitude: locationData.longitude
          };
          robotState.hasLocationData = true;
          console.log(\`Updated location: \${robotState.location.latitude}, \${robotState.location.longitude}\`);
        }
      } catch (e) {
        console.error('Failed to parse location data:', e);
      }
    }
    else if (topic === TOPICS.ALL) {
      try {
        const allData = JSON.parse(payload);
        if (allData.batteryLevel !== undefined) {
          robotState.batteryLevel = allData.batteryLevel;
        }
        if (allData.temperature !== undefined) {
          robotState.temperature = allData.temperature;
        }
        if (allData.status !== undefined) {
          robotState.status = allData.status;
        }
        if (allData.location !== undefined) {
          robotState.location = allData.location;
          robotState.hasLocationData = true;
        }
        console.log('Updated all robot state from combined topic');
        
        // Optionally send telemetry immediately when receiving combined data
        sendTelemetry();
      } catch (e) {
        console.error('Failed to parse all data:', e);
      }
    }
  } catch (error) {
    console.error(\`Error processing message from topic \${topic}:\`, error);
  }
});

client.on('error', (error) => {
  console.error('MQTT client error:', error);
});

// Function to send telemetry to RoboMetrics
async function sendTelemetry() {
  try {
    const telemetryData = {
      robotId: ROBOT_ID,
      batteryLevel: robotState.batteryLevel,
      temperature: robotState.temperature,
      status: robotState.status
    };
    
    // Only include location if we have the data
    if (robotState.hasLocationData) {
      telemetryData.location = robotState.location;
    }
    
    console.log('Sending telemetry to RoboMetrics:', telemetryData);
    
    const response = await axios.post(ROBOMETRICS_ENDPOINT, telemetryData, {
      headers: {
        'Content-Type': 'application/json',
        'api-key': API_KEY
      }
    });
    
    console.log('Telemetry sent successfully:', response.data);
  } catch (error) {
    console.error('Failed to send telemetry:', error);
  }
}

// Handle graceful shutdown
process.on('SIGINT', () => {
  console.log('Closing MQTT connection');
  client.end();
  process.exit(0);
});`;

  const pythonExample = `
import paho.mqtt.client as mqtt
import requests
import json
import time
import signal
import sys
import threading

# MQTT Configuration
MQTT_BROKER = "broker.hivemq.com"  # Public broker for example, use your own in production
MQTT_PORT = 1883
TOPICS = {
    "BATTERY": "robometrics/YOUR_ROBOT_ID/battery",
    "TEMPERATURE": "robometrics/YOUR_ROBOT_ID/temperature",
    "STATUS": "robometrics/YOUR_ROBOT_ID/status",
    "LOCATION": "robometrics/YOUR_ROBOT_ID/location",
    "ALL": "robometrics/YOUR_ROBOT_ID/all"  # Combined topic for all data
}

# RoboMetrics configuration
ROBOMETRICS_ENDPOINT = "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry"
ROBOT_ID = "YOUR_ROBOT_ID"
API_KEY = "YOUR_API_KEY"

# State to hold latest robot telemetry
robot_state = {
    "batteryLevel": 100,
    "temperature": 25.0,
    "status": "OK",
    "location": {
        "latitude": 0,
        "longitude": 0
    },
    "hasLocationData": False
}

# Flag to control the telemetry sending loop
running = True

# MQTT client callbacks
def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code {rc}")
    
    # Subscribe to all topics
    for topic in TOPICS.values():
        client.subscribe(topic)
        print(f"Subscribed to {topic}")

def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode()
    print(f"Received message on {topic}: {payload}")
    
    try:
        # Process message based on topic
        if topic == TOPICS["BATTERY"]:
            robot_state["batteryLevel"] = float(payload)
            print(f"Updated battery level: {robot_state['batteryLevel']}%")
        
        elif topic == TOPICS["TEMPERATURE"]:
            robot_state["temperature"] = float(payload)
            print(f"Updated temperature: {robot_state['temperature']}°C")
        
        elif topic == TOPICS["STATUS"]:
            robot_state["status"] = payload
            print(f"Updated status: {robot_state['status']}")
        
        elif topic == TOPICS["LOCATION"]:
            try:
                location_data = json.loads(payload)
                if "latitude" in location_data and "longitude" in location_data:
                    robot_state["location"] = {
                        "latitude": location_data["latitude"],
                        "longitude": location_data["longitude"]
                    }
                    robot_state["hasLocationData"] = True
                    print(f"Updated location: {robot_state['location']['latitude']}, {robot_state['location']['longitude']}")
            except json.JSONDecodeError as e:
                print(f"Failed to parse location data: {e}")
        
        elif topic == TOPICS["ALL"]:
            try:
                all_data = json.loads(payload)
                if "batteryLevel" in all_data:
                    robot_state["batteryLevel"] = all_data["batteryLevel"]
                if "temperature" in all_data:
                    robot_state["temperature"] = all_data["temperature"]
                if "status" in all_data:
                    robot_state["status"] = all_data["status"]
                if "location" in all_data:
                    robot_state["location"] = all_data["location"]
                    robot_state["hasLocationData"] = True
                print("Updated all robot state from combined topic")
                
                # Optionally send telemetry immediately when receiving combined data
                send_telemetry()
            except json.JSONDecodeError as e:
                print(f"Failed to parse all data: {e}")
    
    except Exception as e:
        print(f"Error processing message from topic {topic}: {e}")

def on_disconnect(client, userdata, rc):
    if rc != 0:
        print(f"Unexpected disconnection. Reconnecting... (code {rc})")
        client.reconnect()

# Function to send telemetry to RoboMetrics
def send_telemetry():
    try:
        telemetry_data = {
            "robotId": ROBOT_ID,
            "batteryLevel": robot_state["batteryLevel"],
            "temperature": robot_state["temperature"],
            "status": robot_state["status"]
        }
        
        # Only include location if we have the data
        if robot_state["hasLocationData"]:
            telemetry_data["location"] = robot_state["location"]
        
        print(f"Sending telemetry to RoboMetrics: {telemetry_data}")
        
        headers = {
            "Content-Type": "application/json",
            "api-key": API_KEY
        }
        
        response = requests.post(
            ROBOMETRICS_ENDPOINT, 
            headers=headers, 
            data=json.dumps(telemetry_data)
        )
        
        if response.status_code == 200:
            print(f"Telemetry sent successfully: {response.json()}")
        else:
            print(f"Failed to send telemetry. Status code: {response.status_code}")
            print(f"Response: {response.text}")
            
    except Exception as e:
        print(f"Error sending telemetry: {e}")

# Function to periodically send telemetry
def telemetry_loop():
    while running:
        send_telemetry()
        time.sleep(60)  # Send every 60 seconds

# Handle graceful shutdown
def signal_handler(sig, frame):
    global running
    print('Shutting down...')
    running = False
    client.disconnect()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Create and configure MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.on_disconnect = on_disconnect

# Connect to broker
print(f"Connecting to MQTT broker: {MQTT_BROKER}")
client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Start the telemetry sending loop in a separate thread
telemetry_thread = threading.Thread(target=telemetry_loop)
telemetry_thread.daemon = True
telemetry_thread.start()

# Start the MQTT client loop
client.loop_forever()`;

  const mqttTopicsDoc = `
# MQTT Topics for RoboMetrics Integration

This document describes the MQTT topics structure for integrating robots with RoboMetrics.

## Topic Structure

All topics use the prefix \`robometrics/{robot_id}/\` where \`{robot_id}\` is your robot's ID in the RoboMetrics platform.

## Standard Topics

| Topic | Description | Payload Format | Example |
|-------|-------------|----------------|---------|
| \`robometrics/{robot_id}/battery\` | Battery level percentage | Float value (0-100) | \`75.5\` |
| \`robometrics/{robot_id}/temperature\` | Temperature in Celsius | Float value | \`28.3\` |
| \`robometrics/{robot_id}/status\` | Robot status | String: "OK", "WARNING", or "ERROR" | \`"OK"\` |
| \`robometrics/{robot_id}/location\` | GPS coordinates | JSON object with latitude and longitude | \`{"latitude": 37.7749, "longitude": -122.4194}\` |
| \`robometrics/{robot_id}/all\` | Combined data | JSON object with all telemetry fields | \`{"batteryLevel": 75.5, "temperature": 28.3, "status": "OK", "location": {"latitude": 37.7749, "longitude": -122.4194}}\` |

## Extended Topics

For robots with additional sensors or data points:

| Topic | Description | Payload Format | Example |
|-------|-------------|----------------|---------|
| \`robometrics/{robot_id}/speed\` | Current speed | Float value in m/s | \`1.5\` |
| \`robometrics/{robot_id}/heading\` | Orientation in degrees | Float value (0-359) | \`175.3\` |
| \`robometrics/{robot_id}/cpu\` | CPU utilization | Float value (0-100) | \`45.2\` |
| \`robometrics/{robot_id}/memory\` | Memory utilization | Float value (0-100) | \`62.7\` |
| \`robometrics/{robot_id}/errors\` | Error messages | JSON array of error objects | \`[{"code": "E001", "message": "Motor overheating"}]\` |

## Publishing Frequency

For optimal performance:
- Battery level, temperature, status: Every 1-5 minutes
- Location: Every 10-30 seconds when moving, less frequently when stationary
- Combined data (\`all\` topic): Every 1-5 minutes

## Quality of Service (QoS)

- Use QoS level 0 for high-frequency telemetry data (location updates)
- Use QoS level 1 for important status changes and alerts`;

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
        <CardTitle>MQTT Integration</CardTitle>
        <CardDescription>
          Connect your robots to RoboMetrics via MQTT. These examples show how to subscribe to MQTT topics and forward the data to our platform.
        </CardDescription>
      </CardHeader>
      <CardContent>
        <Tabs defaultValue="nodejs" value={activeTab} onValueChange={setActiveTab}>
          <TabsList className="mb-4">
            <TabsTrigger value="nodejs">Node.js</TabsTrigger>
            <TabsTrigger value="python">Python</TabsTrigger>
            <TabsTrigger value="topics">MQTT Topics</TabsTrigger>
          </TabsList>
          <TabsContent value="nodejs" className="relative">
            <div className="flex justify-end gap-2 mb-2">
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => copyToClipboard(nodejsExample, "Node.js MQTT")}
              >
                <ClipboardCopy className="h-4 w-4 mr-1" /> Copy Code
              </Button>
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => downloadFile(nodejsExample, "mqtt_bridge.js", "text/plain")}
              >
                <Download className="h-4 w-4 mr-1" /> Download
              </Button>
            </div>
            <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
              {nodejsExample}
            </pre>
          </TabsContent>
          <TabsContent value="python" className="relative">
            <div className="flex justify-end gap-2 mb-2">
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => copyToClipboard(pythonExample, "Python MQTT")}
              >
                <ClipboardCopy className="h-4 w-4 mr-1" /> Copy Code
              </Button>
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => downloadFile(pythonExample, "mqtt_bridge.py", "text/plain")}
              >
                <Download className="h-4 w-4 mr-1" /> Download
              </Button>
            </div>
            <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
              {pythonExample}
            </pre>
          </TabsContent>
          <TabsContent value="topics" className="relative">
            <div className="flex justify-end gap-2 mb-2">
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => copyToClipboard(mqttTopicsDoc, "MQTT Topics")}
              >
                <ClipboardCopy className="h-4 w-4 mr-1" /> Copy Content
              </Button>
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => downloadFile(mqttTopicsDoc, "mqtt_topics.md", "text/plain")}
              >
                <Download className="h-4 w-4 mr-1" /> Download
              </Button>
            </div>
            <div className="p-4 bg-muted rounded-md overflow-x-auto text-xs whitespace-pre-wrap">
              {mqttTopicsDoc}
            </div>
          </TabsContent>
        </Tabs>
      </CardContent>
    </Card>
  );
}
