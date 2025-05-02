
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { useState } from "react";
import { toast } from "@/components/ui/sonner";
import { ClipboardCopy, Download } from "lucide-react";

export function MQTTExample() {
  const [activeTab, setActiveTab] = useState("python");

  const pythonExample = `#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import time
import json
import requests
import os
import logging
from datetime import datetime

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# RoboMetrics configuration
API_ENDPOINT = "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry"
ROBOT_ID = "YOUR_ROBOT_ID"
API_KEY = "YOUR_ROBOT_API_KEY"

# MQTT configuration
MQTT_BROKER = "broker.hivemq.com"  # Public broker for testing, use your own in production
MQTT_PORT = 1883
MQTT_TOPIC_PREFIX = "robot/YOUR_ROBOT_ID/"  # Replace with your robot ID
MQTT_CLIENT_ID = f"robometrics_bridge_{ROBOT_ID}"

# Topics to subscribe to
TOPICS = {
    "battery": MQTT_TOPIC_PREFIX + "battery",
    "temperature": MQTT_TOPIC_PREFIX + "temperature",
    "status": MQTT_TOPIC_PREFIX + "status",
    "location": MQTT_TOPIC_PREFIX + "location",
    # Custom telemetry topics
    "motor_speed": MQTT_TOPIC_PREFIX + "motor_speed",
    "error_count": MQTT_TOPIC_PREFIX + "error_count",
    "arm_position": MQTT_TOPIC_PREFIX + "arm_position"
}

# Store the latest data
telemetry_data = {
    "batteryLevel": 100,
    "temperature": 25.0,
    "status": "OK",
    "location": {
        "latitude": 0.0,
        "longitude": 0.0
    },
    "customTelemetry": {
        "motorSpeed": 0,
        "errorCount": 0,
        "armPosition": "retracted"
    }
}

# MQTT callbacks
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        logger.info("Connected to MQTT broker")
        # Subscribe to all topics
        for topic in TOPICS.values():
            client.subscribe(topic)
            logger.info(f"Subscribed to {topic}")
    else:
        logger.error(f"Failed to connect to MQTT broker, return code: {rc}")

def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode("utf-8")
    logger.info(f"Received message on topic {topic}: {payload}")
    
    try:
        # Process different topics
        if topic == TOPICS["battery"]:
            telemetry_data["batteryLevel"] = float(payload)
        
        elif topic == TOPICS["temperature"]:
            telemetry_data["temperature"] = float(payload)
        
        elif topic == TOPICS["status"]:
            telemetry_data["status"] = payload
        
        elif topic == TOPICS["location"]:
            location_data = json.loads(payload)
            telemetry_data["location"]["latitude"] = location_data.get("latitude", 0.0)
            telemetry_data["location"]["longitude"] = location_data.get("longitude", 0.0)
        
        # Custom telemetry topics
        elif topic == TOPICS["motor_speed"]:
            telemetry_data["customTelemetry"]["motorSpeed"] = int(payload)
        
        elif topic == TOPICS["error_count"]:
            telemetry_data["customTelemetry"]["errorCount"] = int(payload)
        
        elif topic == TOPICS["arm_position"]:
            telemetry_data["customTelemetry"]["armPosition"] = payload
        
    except Exception as e:
        logger.error(f"Error processing message: {e}")

def send_telemetry():
    # Prepare the data for sending
    data = {
        "robotId": ROBOT_ID,
        **telemetry_data,
        "timestamp": datetime.utcnow().isoformat() + "Z"
    }
    
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
        logger.info(f"Telemetry sent, status: {response.status_code}")
        if response.status_code >= 400:
            logger.error(f"API error: {response.text}")
    except Exception as e:
        logger.error(f"Error sending telemetry: {e}")

def main():
    # Create MQTT client
    client = mqtt.Client(MQTT_CLIENT_ID)
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to broker
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except Exception as e:
        logger.error(f"Failed to connect to MQTT broker: {e}")
        return
    
    # Start the loop in a non-blocking way
    client.loop_start()
    
    try:
        # Send telemetry every 60 seconds
        while True:
            time.sleep(60)
            send_telemetry()
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()`;

  const nodeExample = `const mqtt = require('mqtt');
const axios = require('axios');

// RoboMetrics configuration
const API_ENDPOINT = "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry";
const ROBOT_ID = "YOUR_ROBOT_ID";
const API_KEY = "YOUR_ROBOT_API_KEY";

// MQTT configuration
const MQTT_BROKER = "mqtt://broker.hivemq.com";  // Public broker for testing, use your own in production
const MQTT_TOPIC_PREFIX = \`robot/\${ROBOT_ID}/\`;
const MQTT_CLIENT_ID = \`robometrics_bridge_\${ROBOT_ID}_\${Math.random().toString(16).slice(2, 8)}\`;

// Topics to subscribe to
const TOPICS = {
  battery: MQTT_TOPIC_PREFIX + "battery",
  temperature: MQTT_TOPIC_PREFIX + "temperature",
  status: MQTT_TOPIC_PREFIX + "status",
  location: MQTT_TOPIC_PREFIX + "location",
  // Custom telemetry topics
  motorSpeed: MQTT_TOPIC_PREFIX + "motor_speed",
  errorCount: MQTT_TOPIC_PREFIX + "error_count",
  armPosition: MQTT_TOPIC_PREFIX + "arm_position"
};

// Store the latest data
const telemetryData = {
  batteryLevel: 100,
  temperature: 25.0,
  status: "OK",
  location: {
    latitude: 0.0,
    longitude: 0.0
  },
  customTelemetry: {
    motorSpeed: 0,
    errorCount: 0,
    armPosition: "retracted"
  }
};

// Connect to MQTT broker
console.log("Connecting to MQTT broker...");
const client = mqtt.connect(MQTT_BROKER, {
  clientId: MQTT_CLIENT_ID,
  clean: true
});

client.on('connect', () => {
  console.log("Connected to MQTT broker");
  
  // Subscribe to all topics
  Object.values(TOPICS).forEach(topic => {
    client.subscribe(topic, (err) => {
      if (!err) {
        console.log(\`Subscribed to \${topic}\`);
      } else {
        console.error(\`Error subscribing to \${topic}: \${err.message}\`);
      }
    });
  });
});

client.on('message', (topic, message) => {
  const payload = message.toString();
  console.log(\`Received message on topic \${topic}: \${payload}\`);
  
  try {
    // Process different topics
    if (topic === TOPICS.battery) {
      telemetryData.batteryLevel = parseFloat(payload);
    }
    else if (topic === TOPICS.temperature) {
      telemetryData.temperature = parseFloat(payload);
    }
    else if (topic === TOPICS.status) {
      telemetryData.status = payload;
    }
    else if (topic === TOPICS.location) {
      const locationData = JSON.parse(payload);
      telemetryData.location.latitude = locationData.latitude || 0.0;
      telemetryData.location.longitude = locationData.longitude || 0.0;
    }
    // Custom telemetry topics
    else if (topic === TOPICS.motorSpeed) {
      telemetryData.customTelemetry.motorSpeed = parseInt(payload, 10);
    }
    else if (topic === TOPICS.errorCount) {
      telemetryData.customTelemetry.errorCount = parseInt(payload, 10);
    }
    else if (topic === TOPICS.armPosition) {
      telemetryData.customTelemetry.armPosition = payload;
    }
  } catch (error) {
    console.error(\`Error processing message: \${error.message}\`);
  }
});

client.on('error', (error) => {
  console.error(\`MQTT error: \${error.message}\`);
});

// Send telemetry to RoboMetrics
async function sendTelemetry() {
  const data = {
    robotId: ROBOT_ID,
    ...telemetryData,
    timestamp: new Date().toISOString()
  };
  
  const headers = {
    'Content-Type': 'application/json',
    'api-key': API_KEY
  };
  
  try {
    const response = await axios.post(API_ENDPOINT, data, { headers });
    console.log(\`Telemetry sent, status: \${response.status}\`);
  } catch (error) {
    console.error(\`Error sending telemetry: \${error.message}\`);
    if (error.response) {
      console.error(\`API error: \${JSON.stringify(error.response.data)}\`);
    }
  }
}

// Send telemetry every 60 seconds
setInterval(sendTelemetry, 60000);

// Handle shutdown
process.on('SIGINT', () => {
  console.log('Shutting down...');
  client.end();
  process.exit();
});`;

  const espExample = `
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

// WiFi configuration
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// MQTT configuration
const char* MQTT_BROKER = "broker.hivemq.com";  // Public broker for testing, use your own in production
const int MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "esp32_robometrics_bridge";

// RoboMetrics configuration
const char* API_ENDPOINT = "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry";
const char* ROBOT_ID = "YOUR_ROBOT_ID";
const char* API_KEY = "YOUR_ROBOT_API_KEY";

// MQTT topics to subscribe to (incoming data)
const char* TOPIC_BATTERY = "robot/incoming/battery";
const char* TOPIC_TEMPERATURE = "robot/incoming/temperature";
const char* TOPIC_STATUS = "robot/incoming/status";
const char* TOPIC_LOCATION = "robot/incoming/location";
// Custom telemetry topics
const char* TOPIC_MOTOR_SPEED = "robot/incoming/motor_speed";
const char* TOPIC_ERROR_COUNT = "robot/incoming/error_count";
const char* TOPIC_ARM_POSITION = "robot/incoming/arm_position";

// Sensor data
float batteryLevel = 100.0;
float temperature = 25.0;
float latitude = 37.7749;
float longitude = -122.4194;
String status = "OK";
// Custom telemetry data
int motorSpeed = 0;
int errorCount = 0;
String armPosition = "retracted";

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Timer for sending telemetry
unsigned long lastTelemetrySent = 0;
const unsigned long TELEMETRY_INTERVAL = 60000; // 60 seconds

// Connect to WiFi
void setupWifi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// MQTT message callback
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);
  
  // Convert payload to string
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\\0';
  
  Serial.print("Payload: ");
  Serial.println(message);
  
  // Parse message based on topic
  if (strcmp(topic, TOPIC_BATTERY) == 0) {
    batteryLevel = atof(message);
  } 
  else if (strcmp(topic, TOPIC_TEMPERATURE) == 0) {
    temperature = atof(message);
  }
  else if (strcmp(topic, TOPIC_STATUS) == 0) {
    status = String(message);
  }
  else if (strcmp(topic, TOPIC_LOCATION) == 0) {
    // Parse JSON location
    DynamicJsonDocument doc(200);
    deserializeJson(doc, message);
    latitude = doc["latitude"] | 0.0;
    longitude = doc["longitude"] | 0.0;
  }
  // Custom telemetry
  else if (strcmp(topic, TOPIC_MOTOR_SPEED) == 0) {
    motorSpeed = atoi(message);
  }
  else if (strcmp(topic, TOPIC_ERROR_COUNT) == 0) {
    errorCount = atoi(message);
  }
  else if (strcmp(topic, TOPIC_ARM_POSITION) == 0) {
    armPosition = String(message);
  }
}

void reconnectMqtt() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT broker...");
    if (mqttClient.connect(MQTT_CLIENT_ID)) {
      Serial.println("connected");
      
      // Subscribe to topics
      mqttClient.subscribe(TOPIC_BATTERY);
      mqttClient.subscribe(TOPIC_TEMPERATURE);
      mqttClient.subscribe(TOPIC_STATUS);
      mqttClient.subscribe(TOPIC_LOCATION);
      // Subscribe to custom telemetry topics
      mqttClient.subscribe(TOPIC_MOTOR_SPEED);
      mqttClient.subscribe(TOPIC_ERROR_COUNT);
      mqttClient.subscribe(TOPIC_ARM_POSITION);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

void sendTelemetry() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected. Reconnecting...");
    setupWifi();
    return;
  }
  
  HTTPClient http;
  http.begin(API_ENDPOINT);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("api-key", API_KEY);
  
  // Create JSON document
  DynamicJsonDocument doc(1024);
  doc["robotId"] = ROBOT_ID;
  doc["batteryLevel"] = batteryLevel;
  doc["temperature"] = temperature;
  doc["status"] = status;
  
  JsonObject location = doc.createNestedObject("location");
  location["latitude"] = latitude;
  location["longitude"] = longitude;
  
  // Add custom telemetry
  JsonObject customTelemetry = doc.createNestedObject("customTelemetry");
  customTelemetry["motorSpeed"] = motorSpeed;
  customTelemetry["errorCount"] = errorCount;
  customTelemetry["armPosition"] = armPosition;
  
  String requestBody;
  serializeJson(doc, requestBody);
  
  Serial.println("Sending telemetry data...");
  Serial.println(requestBody);
  
  int httpResponseCode = http.POST(requestBody);
  
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("HTTP Response code: " + String(httpResponseCode));
    Serial.println(response);
  } else {
    Serial.print("Error on sending request: ");
    Serial.println(httpResponseCode);
  }
  
  http.end();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Connect to WiFi
  setupWifi();
  
  // Configure MQTT
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
}

void loop() {
  // Ensure MQTT connection
  if (!mqttClient.connected()) {
    reconnectMqtt();
  }
  mqttClient.loop();
  
  // Send telemetry at regular intervals
  unsigned long currentMillis = millis();
  if (currentMillis - lastTelemetrySent >= TELEMETRY_INTERVAL) {
    lastTelemetrySent = currentMillis;
    sendTelemetry();
  }
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
        <CardTitle>MQTT Integration</CardTitle>
        <CardDescription>
          Connect your robots through MQTT, a lightweight messaging protocol ideal for IoT devices. These examples demonstrate how to bridge MQTT messages to the RoboMetrics platform.
        </CardDescription>
      </CardHeader>
      <CardContent>
        <Tabs defaultValue="python" value={activeTab} onValueChange={setActiveTab}>
          <TabsList className="mb-4">
            <TabsTrigger value="python">Python</TabsTrigger>
            <TabsTrigger value="node">Node.js</TabsTrigger>
            <TabsTrigger value="esp">ESP32/Arduino</TabsTrigger>
          </TabsList>
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
          <TabsContent value="node" className="relative">
            <div className="flex justify-end gap-2 mb-2">
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => copyToClipboard(nodeExample, "Node.js MQTT")}
              >
                <ClipboardCopy className="h-4 w-4 mr-1" /> Copy Code
              </Button>
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => downloadFile(nodeExample, "mqtt_bridge.js", "text/plain")}
              >
                <Download className="h-4 w-4 mr-1" /> Download
              </Button>
            </div>
            <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
              {nodeExample}
            </pre>
          </TabsContent>
          <TabsContent value="esp" className="relative">
            <div className="flex justify-end gap-2 mb-2">
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => copyToClipboard(espExample, "ESP32 MQTT")}
              >
                <ClipboardCopy className="h-4 w-4 mr-1" /> Copy Code
              </Button>
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => downloadFile(espExample, "mqtt_esp32_bridge.ino", "text/plain")}
              >
                <Download className="h-4 w-4 mr-1" /> Download
              </Button>
            </div>
            <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
              {espExample}
            </pre>
          </TabsContent>
        </Tabs>
        <div className="mt-6 bg-muted-foreground/10 p-4 rounded-md">
          <h3 className="text-md font-semibold mb-2">MQTT Topic Structure</h3>
          <p className="text-sm mb-4">
            These examples use the following MQTT topic structure for standard and custom telemetry:
          </p>
          <ul className="list-disc pl-6 space-y-1 text-sm">
            <li><code>robot/YOUR_ROBOT_ID/battery</code> - Battery level (0-100)</li>
            <li><code>robot/YOUR_ROBOT_ID/temperature</code> - Temperature in Celsius</li>
            <li><code>robot/YOUR_ROBOT_ID/status</code> - Robot status (OK, WARNING, ERROR)</li>
            <li><code>robot/YOUR_ROBOT_ID/location</code> - JSON object with latitude and longitude</li>
            <li><code>robot/YOUR_ROBOT_ID/motor_speed</code> - Custom telemetry for motor speed</li>
            <li><code>robot/YOUR_ROBOT_ID/error_count</code> - Custom telemetry for error count</li>
            <li><code>robot/YOUR_ROBOT_ID/arm_position</code> - Custom telemetry for arm position state</li>
          </ul>
        </div>
      </CardContent>
    </Card>
  );
}
