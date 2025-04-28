
import { MainLayout } from "@/components/layout/MainLayout";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Alert, AlertDescription, AlertTitle } from "@/components/ui/alert";
import { Info, ArrowRight, Terminal, Code } from "lucide-react";
import { Button } from "@/components/ui/button";
import { useToast } from "@/hooks/use-toast";
import { supabase } from "@/integrations/supabase/client";
import { Link } from "react-router-dom";

const IntegrationGuide = () => {
  const { toast } = useToast();
  const apiBaseUrl = "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1";
  
  const copyToClipboard = (text: string, message: string) => {
    navigator.clipboard.writeText(text);
    toast({
      title: "Copied!",
      description: message,
    });
  };
  
  const testEndpoint = async () => {
    toast({
      title: "Testing endpoint...",
      description: "Sending sample telemetry data",
    });
    
    try {
      // Just testing the endpoint, not actually sending real data
      const response = await fetch(`${apiBaseUrl}/telemetry`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'api-key': 'test-api-key'
        },
        body: JSON.stringify({
          robotId: "00000000-0000-0000-0000-000000000000", // Dummy ID
          batteryLevel: 80,
          temperature: 25.5,
          status: "OK",
          location: { lat: 37.7749, lng: -122.4194 },
          timestamp: new Date().toISOString()
        }),
      });
      
      const data = await response.json();
      
      if (response.ok) {
        toast({
          title: "Success!",
          description: "Endpoint is working correctly",
        });
      } else {
        toast({
          title: "Error",
          description: data.error || "Something went wrong",
          variant: "destructive",
        });
      }
    } catch (error) {
      toast({
        title: "Connection Error",
        description: "Could not reach the API endpoint",
        variant: "destructive",
      });
    }
  };
  
  return (
    <MainLayout>
      <div className="container mx-auto py-6">
        <div className="flex justify-between items-center mb-6">
          <h1 className="text-3xl font-bold">RoboMetrics API Integration Guide</h1>
          <Link to="/dashboard">
            <Button variant="outline">
              Back to Dashboard <ArrowRight className="ml-2 h-4 w-4" />
            </Button>
          </Link>
        </div>
        
        <Alert className="mb-6 bg-blue-50">
          <Info className="h-4 w-4" />
          <AlertTitle>Getting Started</AlertTitle>
          <AlertDescription>
            Welcome to RoboMetrics! Follow this guide to integrate your robots with our platform.
          </AlertDescription>
        </Alert>
        
        <div className="mb-6">
          <Card>
            <CardHeader>
              <CardTitle>API Base URL</CardTitle>
              <CardDescription>
                All API requests are made to this base URL
              </CardDescription>
            </CardHeader>
            <CardContent>
              <div className="bg-muted p-3 rounded-md font-mono text-sm mb-3">
                {apiBaseUrl}
              </div>
              <Button 
                variant="secondary" 
                onClick={() => copyToClipboard(
                  apiBaseUrl, 
                  "API base URL copied to clipboard"
                )}
              >
                Copy Base URL
              </Button>
            </CardContent>
          </Card>
        </div>

        <div className="mb-6">
          <Card>
            <CardHeader>
              <CardTitle>Authentication</CardTitle>
              <CardDescription>
                All requests require an API Key provided in the request header
              </CardDescription>
            </CardHeader>
            <CardContent>
              <div className="bg-muted p-3 rounded-md font-mono text-sm mb-3">
                <pre>{"{\n  \"api-key\": \"YOUR_ROBOT_API_KEY\",\n  \"Content-Type\": \"application/json\"\n}"}</pre>
              </div>
              <p className="text-sm text-muted-foreground mb-4">
                You can find your API key in the robot settings page after registering your robot in the dashboard.
              </p>
            </CardContent>
          </Card>
        </div>
        
        <Tabs defaultValue="submit" className="mb-6">
          <TabsList className="mb-2">
            <TabsTrigger value="submit">Submit Telemetry</TabsTrigger>
            <TabsTrigger value="get">Get Telemetry</TabsTrigger>
          </TabsList>
          
          <TabsContent value="submit">
            <Card>
              <CardHeader className="flex flex-row items-center justify-between">
                <div>
                  <CardTitle>1. Submit Telemetry</CardTitle>
                  <CardDescription className="mt-1">
                    Send telemetry data from your robot to our platform
                  </CardDescription>
                </div>
                <Terminal className="h-5 w-5 text-muted-foreground" />
              </CardHeader>
              <CardContent>
                <div className="space-y-4">
                  <div>
                    <h3 className="text-sm font-medium mb-1">Endpoint</h3>
                    <div className="bg-muted p-2 rounded-md font-mono text-sm">
                      POST {apiBaseUrl}/telemetry
                    </div>
                  </div>
                  
                  <div>
                    <h3 className="text-sm font-medium mb-1">Request Body</h3>
                    <pre className="bg-muted p-3 rounded-md font-mono text-sm overflow-auto">
{`{
  "robotId": "robot-001",
  "batteryLevel": 87,        // battery percentage
  "temperature": 26.5,       // Celsius
  "status": "OK",            // OK / WARNING / ERROR
  "location": {
    "lat": 33.7756,
    "lng": -84.3963
  },
  "timestamp": "2025-04-28T20:22:00Z"
}`}
                    </pre>
                  </div>
                  
                  <div>
                    <h3 className="text-sm font-medium mb-1">CURL Example</h3>
                    <pre className="bg-muted p-3 rounded-md font-mono text-sm overflow-auto whitespace-pre-wrap">
{`curl -X POST ${apiBaseUrl}/telemetry \\
-H "api-key: YOUR_ROBOT_API_KEY" \\
-H "Content-Type: application/json" \\
-d '{
  "robotId": "robot-001",
  "batteryLevel": 90,
  "temperature": 24.3,
  "status": "OK",
  "location": { "lat": 40.7128, "lng": -74.0060 },
  "timestamp": "2025-04-28T15:00:00Z"
}'`}
                    </pre>
                  </div>

                  <Button 
                    variant="secondary"
                    onClick={() => copyToClipboard(
                      document.querySelector('pre')?.innerText || "", 
                      "CURL example copied to clipboard"
                    )}
                  >
                    Copy CURL Example
                  </Button>
                </div>
              </CardContent>
            </Card>
          </TabsContent>
          
          <TabsContent value="get">
            <Card>
              <CardHeader className="flex flex-row items-center justify-between">
                <div>
                  <CardTitle>2. Get Robot Telemetry</CardTitle>
                  <CardDescription className="mt-1">
                    Retrieve telemetry history for a specific robot
                  </CardDescription>
                </div>
                <Code className="h-5 w-5 text-muted-foreground" />
              </CardHeader>
              <CardContent>
                <div className="space-y-4">
                  <div>
                    <h3 className="text-sm font-medium mb-1">Endpoint</h3>
                    <div className="bg-muted p-2 rounded-md font-mono text-sm">
                      GET {apiBaseUrl}/get-telemetry
                    </div>
                  </div>
                  
                  <div>
                    <h3 className="text-sm font-medium mb-1">URL Format</h3>
                    <div className="bg-muted p-2 rounded-md font-mono text-sm">
                      /v1/robots/{"{robotId}"}/telemetry?last={"{count}"}
                    </div>
                    <p className="text-sm text-muted-foreground mt-1">
                      The "last" parameter is optional and defaults to 100
                    </p>
                  </div>

                  <div>
                    <h3 className="text-sm font-medium mb-1">CURL Example</h3>
                    <pre className="bg-muted p-3 rounded-md font-mono text-sm overflow-auto whitespace-pre-wrap">
{`curl -X GET "${apiBaseUrl}/v1/robots/robot-001/telemetry?last=100" \\
-H "api-key: YOUR_ROBOT_API_KEY"`}
                    </pre>
                  </div>

                  <Button 
                    variant="secondary"
                    onClick={() => copyToClipboard(
                      `curl -X GET "${apiBaseUrl}/v1/robots/robot-001/telemetry?last=100" -H "api-key: YOUR_ROBOT_API_KEY"`, 
                      "CURL example copied to clipboard"
                    )}
                  >
                    Copy CURL Example
                  </Button>
                </div>
              </CardContent>
            </Card>
          </TabsContent>
        </Tabs>
        
        <h2 className="text-2xl font-semibold mb-4 mt-8">RoboMetrics Integration Guide</h2>
        
        <div className="grid gap-6 md:grid-cols-3 mb-6">
          <Card>
            <CardHeader>
              <CardTitle>Step 1: Get Your API Key</CardTitle>
            </CardHeader>
            <CardContent className="space-y-2">
              <p>1. Sign up at RoboMetrics</p>
              <p>2. Create your Robot Device in Dashboard</p>
              <p>3. Copy your unique API Key</p>
            </CardContent>
          </Card>
          
          <Card>
            <CardHeader>
              <CardTitle>Step 2: Add Telemetry Code</CardTitle>
            </CardHeader>
            <CardContent className="space-y-2">
              <p>1. Choose your robot platform</p>
              <p>2. Implement telemetry sending code</p>
              <p>3. Send data every 10-30 seconds</p>
            </CardContent>
          </Card>
          
          <Card>
            <CardHeader>
              <CardTitle>Step 3: View in Dashboard</CardTitle>
            </CardHeader>
            <CardContent className="space-y-2">
              <p>1. Log in to RoboMetrics</p>
              <p>2. See robot live data</p>
              <p>3. Set up alerts for critical events</p>
            </CardContent>
          </Card>
        </div>
        
        <Tabs defaultValue="arduino" className="mb-6">
          <TabsList className="mb-2">
            <TabsTrigger value="arduino">Arduino/ESP32</TabsTrigger>
            <TabsTrigger value="python">Python/Raspberry Pi</TabsTrigger>
            <TabsTrigger value="postman">Postman</TabsTrigger>
          </TabsList>
          
          <TabsContent value="arduino">
            <Card>
              <CardHeader>
                <CardTitle>Arduino/ESP32 Example</CardTitle>
                <CardDescription>
                  Use this code snippet to send telemetry data from your Arduino device
                </CardDescription>
              </CardHeader>
              <CardContent>
                <pre className="bg-muted p-4 rounded-md overflow-auto text-sm">
{`#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* apiUrl = "${apiBaseUrl}/telemetry";
const char* robotId = "YOUR_ROBOT_ID";
const char* apiKey = "YOUR_API_KEY";

void setup() {
  Serial.begin(9600);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  Serial.println("Connected to WiFi");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    sendTelemetry(getBatteryLevel(), getTemperature());
  }
  delay(10000); // Send every 10 seconds
}

void sendTelemetry(int battery, float temp) {
  HTTPClient http;
  http.begin(apiUrl);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("api-key", apiKey);

  // Create JSON document
  StaticJsonDocument<200> doc;
  doc["robotId"] = robotId;
  doc["batteryLevel"] = battery;
  doc["temperature"] = temp;
  doc["status"] = "OK";
  
  // Add location if available
  JsonObject location = doc.createNestedObject("location");
  location["lat"] = 37.7749;
  location["lng"] = -122.4194;
  
  // Add timestamp
  char timestamp[25];
  // You would implement a proper timestamp generator here
  strcpy(timestamp, "2025-04-28T20:00:00Z");
  doc["timestamp"] = timestamp;
  
  // Serialize JSON to String
  String payload;
  serializeJson(doc, payload);
  
  int httpCode = http.POST(payload);
  
  if (httpCode > 0) {
    String response = http.getString();
    Serial.println(httpCode);
    Serial.println(response);
  } else {
    Serial.println("Error sending telemetry");
  }
  
  http.end();
}

float getBatteryLevel() {
  // Replace with your actual code to read battery level
  return 75.0;
}

float getTemperature() {
  // Replace with your actual code to read temperature
  return 24.5;
}`}
                </pre>
                <Button 
                  variant="secondary" 
                  className="mt-2"
                  onClick={() => copyToClipboard(
                    document.querySelector('pre')?.innerText || "", 
                    "Arduino code copied to clipboard"
                  )}
                >
                  Copy Code
                </Button>
              </CardContent>
            </Card>
          </TabsContent>
          
          <TabsContent value="python">
            <Card>
              <CardHeader>
                <CardTitle>Python Example</CardTitle>
                <CardDescription>
                  Use this Python script to send telemetry data
                </CardDescription>
              </CardHeader>
              <CardContent>
                <pre className="bg-muted p-4 rounded-md overflow-auto text-sm">
{`import requests
import json
import time
from datetime import datetime

# Configuration
ROBOT_ID = "YOUR_ROBOT_ID"
API_KEY = "YOUR_API_KEY"
API_URL = "${apiBaseUrl}/telemetry"

def get_battery_level():
    # Replace with actual code to get battery level
    return 85.0

def get_temperature():
    # Replace with actual code to get temperature
    return 23.5

def get_location():
    # Replace with actual code to get GPS coordinates
    return {"lat": 37.7749, "lng": -122.4194}

def send_telemetry():
    # Prepare telemetry data
    data = {
        "robotId": ROBOT_ID,
        "batteryLevel": get_battery_level(),
        "temperature": get_temperature(),
        "status": "OK",
        "location": get_location(),
        "timestamp": datetime.now().isoformat() + "Z"
    }
    
    # Send data
    try:
        headers = {
            "api-key": API_KEY,
            "Content-Type": "application/json"
        }
        response = requests.post(API_URL, json=data, headers=headers)
        print(f"Status Code: {response.status_code}")
        print(f"Response: {response.json()}")
        return response.status_code == 200
    except Exception as e:
        print(f"Error sending telemetry: {e}")
        return False

if __name__ == "__main__":
    while True:
        print("Sending telemetry data...")
        success = send_telemetry()
        print(f"Telemetry sent: {'Success' if success else 'Failed'}")
        time.sleep(10)  # Send data every 10 seconds`}
                </pre>
                <Button 
                  variant="secondary" 
                  className="mt-2"
                  onClick={() => copyToClipboard(
                    document.querySelector('pre')?.innerText || "", 
                    "Python code copied to clipboard"
                  )}
                >
                  Copy Code
                </Button>
              </CardContent>
            </Card>
          </TabsContent>
          
          <TabsContent value="postman">
            <Card>
              <CardHeader>
                <CardTitle>Postman Example</CardTitle>
                <CardDescription>
                  Use this example to test the API with Postman
                </CardDescription>
              </CardHeader>
              <CardContent>
                <div className="space-y-4">
                  <div>
                    <h3 className="text-sm font-medium mb-1">URL</h3>
                    <div className="bg-muted p-2 rounded-md font-mono text-sm">
                      {apiBaseUrl}/telemetry
                    </div>
                  </div>
                  
                  <div>
                    <h3 className="text-sm font-medium mb-1">Method</h3>
                    <div className="bg-muted p-2 rounded-md font-mono text-sm">
                      POST
                    </div>
                  </div>
                  
                  <div>
                    <h3 className="text-sm font-medium mb-1">Headers</h3>
                    <div className="bg-muted p-2 rounded-md font-mono text-sm">
                      Content-Type: application/json<br />
                      api-key: YOUR_ROBOT_API_KEY
                    </div>
                  </div>
                  
                  <div>
                    <h3 className="text-sm font-medium mb-1">Body (raw JSON)</h3>
                    <pre className="bg-muted p-2 rounded-md font-mono text-sm overflow-auto">
{`{
  "robotId": "YOUR_ROBOT_ID",
  "batteryLevel": 90,
  "temperature": 22.5,
  "status": "OK",
  "location": {
    "lat": 37.7749,
    "lng": -122.4194
  },
  "timestamp": "2025-04-28T20:00:00Z"
}`}
                    </pre>
                  </div>
                  
                  <Button 
                    variant="secondary"
                    onClick={() => copyToClipboard(
                      document.querySelector('pre')?.innerText || "", 
                      "JSON payload copied to clipboard"
                    )}
                  >
                    Copy JSON Payload
                  </Button>
                </div>
              </CardContent>
            </Card>
          </TabsContent>
        </Tabs>
        
        <Card className="mb-6">
          <CardHeader>
            <CardTitle>API Documentation</CardTitle>
            <CardDescription>
              Detailed information about the telemetry API
            </CardDescription>
          </CardHeader>
          <CardContent>
            <div className="space-y-4">
              <div>
                <h3 className="text-lg font-medium mb-2">Request Parameters</h3>
                <table className="w-full border-collapse">
                  <thead>
                    <tr className="bg-muted">
                      <th className="border border-border p-2 text-left">Parameter</th>
                      <th className="border border-border p-2 text-left">Type</th>
                      <th className="border border-border p-2 text-left">Required</th>
                      <th className="border border-border p-2 text-left">Description</th>
                    </tr>
                  </thead>
                  <tbody>
                    <tr>
                      <td className="border border-border p-2">robotId</td>
                      <td className="border border-border p-2">String</td>
                      <td className="border border-border p-2">Yes</td>
                      <td className="border border-border p-2">The unique ID of your robot</td>
                    </tr>
                    <tr>
                      <td className="border border-border p-2">batteryLevel</td>
                      <td className="border border-border p-2">Number</td>
                      <td className="border border-border p-2">No</td>
                      <td className="border border-border p-2">Battery level percentage (0-100)</td>
                    </tr>
                    <tr>
                      <td className="border border-border p-2">temperature</td>
                      <td className="border border-border p-2">Number</td>
                      <td className="border border-border p-2">No</td>
                      <td className="border border-border p-2">Temperature in Celsius</td>
                    </tr>
                    <tr>
                      <td className="border border-border p-2">status</td>
                      <td className="border border-border p-2">String</td>
                      <td className="border border-border p-2">No</td>
                      <td className="border border-border p-2">"OK", "WARNING", or "ERROR"</td>
                    </tr>
                    <tr>
                      <td className="border border-border p-2">location</td>
                      <td className="border border-border p-2">Object</td>
                      <td className="border border-border p-2">No</td>
                      <td className="border border-border p-2">Object with lat and lng properties</td>
                    </tr>
                    <tr>
                      <td className="border border-border p-2">timestamp</td>
                      <td className="border border-border p-2">String</td>
                      <td className="border border-border p-2">No</td>
                      <td className="border border-border p-2">ISO 8601 formatted date-time string</td>
                    </tr>
                  </tbody>
                </table>
              </div>
              
              <div>
                <h3 className="text-lg font-medium mb-2">Response</h3>
                <pre className="bg-muted p-3 rounded-md font-mono text-sm">
{`// Success
{
  "success": true,
  "message": "Telemetry data received"
}

// Error
{
  "error": "Error message",
  "details": {}
}`}
                </pre>
              </div>
            </div>
          </CardContent>
        </Card>
        
        <div className="flex justify-between">
          <Button variant="outline" asChild>
            <Link to="/">Back to Home</Link>
          </Button>
          <Button>
            <Link to="/dashboard">View Your Robots</Link>
          </Button>
        </div>
      </div>
    </MainLayout>
  );
};

export default IntegrationGuide;
