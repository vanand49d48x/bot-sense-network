
import { MainLayout } from "@/components/layout/MainLayout";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Alert, AlertDescription, AlertTitle } from "@/components/ui/alert";
import { Info, ArrowRight } from "lucide-react";
import { Button } from "@/components/ui/button";
import { useToast } from "@/hooks/use-toast";
import { supabase } from "@/integrations/supabase/client";
import { Link } from "react-router-dom";

const IntegrationGuide = () => {
  const { toast } = useToast();
  const supabaseUrl = "https://uwmbdporlrduzthgdmcg.supabase.co";
  
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
      const response = await fetch(`${supabaseUrl}/functions/v1/telemetry`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          robotId: "00000000-0000-0000-0000-000000000000", // Dummy ID
          apiKey: "test-key",
          battery_level: 80,
          temperature: 25.5,
          location: { latitude: 37.7749, longitude: -122.4194 }
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
          <h1 className="text-3xl font-bold">Robot Integration Guide</h1>
          <Link to="/">
            <Button variant="outline">
              Back to Dashboard <ArrowRight className="ml-2 h-4 w-4" />
            </Button>
          </Link>
        </div>
        
        <Alert className="mb-6 bg-blue-50">
          <Info className="h-4 w-4" />
          <AlertTitle>Getting Started</AlertTitle>
          <AlertDescription>
            Follow this guide to integrate your robots with our platform using the Telemetry API.
          </AlertDescription>
        </Alert>
        
        <div className="mb-6">
          <Card>
            <CardHeader>
              <CardTitle>API Endpoint</CardTitle>
              <CardDescription>
                Send telemetry data to this endpoint using HTTP POST requests
              </CardDescription>
            </CardHeader>
            <CardContent>
              <div className="bg-muted p-3 rounded-md font-mono text-sm mb-3">
                POST {supabaseUrl}/functions/v1/telemetry
              </div>
              <Button 
                variant="secondary" 
                onClick={() => copyToClipboard(
                  `${supabaseUrl}/functions/v1/telemetry`, 
                  "API endpoint copied to clipboard"
                )}
              >
                Copy Endpoint
              </Button>
              <Button 
                variant="outline" 
                onClick={testEndpoint} 
                className="ml-2"
              >
                Test Endpoint
              </Button>
            </CardContent>
          </Card>
        </div>
        
        <Tabs defaultValue="arduino" className="mb-6">
          <TabsList className="mb-2">
            <TabsTrigger value="arduino">Arduino</TabsTrigger>
            <TabsTrigger value="python">Python</TabsTrigger>
            <TabsTrigger value="postman">Postman</TabsTrigger>
          </TabsList>
          
          <TabsContent value="arduino">
            <Card>
              <CardHeader>
                <CardTitle>Arduino Example</CardTitle>
                <CardDescription>
                  Use this code snippet to send telemetry data from your Arduino device
                </CardDescription>
              </CardHeader>
              <CardContent>
                <pre className="bg-muted p-4 rounded-md overflow-auto text-sm">
{`#include <ArduinoJson.h>
#include <WiFiNINA.h> // or ESP8266WiFi.h for ESP modules

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* serverUrl = "${supabaseUrl}/functions/v1/telemetry";
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
    // Create JSON document
    StaticJsonDocument<200> doc;
    doc["robotId"] = robotId;
    doc["apiKey"] = apiKey;
    doc["battery_level"] = getBatteryLevel();
    doc["temperature"] = getTemperature();
    
    // Serialize JSON to string
    String jsonString;
    serializeJson(doc, jsonString);
    
    // Send HTTP POST request
    WiFiClient client;
    HTTPClient http;
    
    http.begin(client, serverUrl);
    http.addHeader("Content-Type", "application/json");
    int httpCode = http.POST(jsonString);
    
    if (httpCode > 0) {
      String response = http.getString();
      Serial.println(httpCode);
      Serial.println(response);
    } else {
      Serial.println("Error on HTTP request");
    }
    
    http.end();
  }
  
  delay(60000); // Send data every minute
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
API_URL = "${supabaseUrl}/functions/v1/telemetry"

def get_battery_level():
    # Replace with actual code to get battery level
    return 85.0

def get_temperature():
    # Replace with actual code to get temperature
    return 23.5

def get_location():
    # Replace with actual code to get GPS coordinates
    return {"latitude": 37.7749, "longitude": -122.4194}

def send_telemetry():
    # Prepare telemetry data
    data = {
        "robotId": ROBOT_ID,
        "apiKey": API_KEY,
        "battery_level": get_battery_level(),
        "temperature": get_temperature(),
        "location": get_location(),
        "timestamp": datetime.now().isoformat()
    }
    
    # Send data
    try:
        response = requests.post(API_URL, json=data)
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
        time.sleep(60)  # Send data every minute`}
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
                      {supabaseUrl}/functions/v1/telemetry
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
                      Content-Type: application/json
                    </div>
                  </div>
                  
                  <div>
                    <h3 className="text-sm font-medium mb-1">Body (raw JSON)</h3>
                    <pre className="bg-muted p-2 rounded-md font-mono text-sm overflow-auto">
{`{
  "robotId": "YOUR_ROBOT_ID",
  "apiKey": "YOUR_API_KEY",
  "battery_level": 90,
  "temperature": 22.5,
  "location": {
    "latitude": 37.7749,
    "longitude": -122.4194
  },
  "motor_status": {
    "left_motor": "ok",
    "right_motor": "ok"
  },
  "error_codes": []
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
                      <td className="border border-border p-2">apiKey</td>
                      <td className="border border-border p-2">String</td>
                      <td className="border border-border p-2">Yes</td>
                      <td className="border border-border p-2">Your API key for authentication</td>
                    </tr>
                    <tr>
                      <td className="border border-border p-2">battery_level</td>
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
                      <td className="border border-border p-2">location</td>
                      <td className="border border-border p-2">Object</td>
                      <td className="border border-border p-2">No</td>
                      <td className="border border-border p-2">Object with latitude and longitude</td>
                    </tr>
                    <tr>
                      <td className="border border-border p-2">motor_status</td>
                      <td className="border border-border p-2">Object</td>
                      <td className="border border-border p-2">No</td>
                      <td className="border border-border p-2">Object with motor status details</td>
                    </tr>
                    <tr>
                      <td className="border border-border p-2">error_codes</td>
                      <td className="border border-border p-2">Array</td>
                      <td className="border border-border p-2">No</td>
                      <td className="border border-border p-2">Array of error codes</td>
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
            <Link to="/">Back to Dashboard</Link>
          </Button>
          <Button>
            <Link to="/">View Your Robots</Link>
          </Button>
        </div>
      </div>
    </MainLayout>
  );
};

export default IntegrationGuide;
