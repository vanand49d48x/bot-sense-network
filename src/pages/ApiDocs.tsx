import React, { useEffect, useState } from "react";
import PlaceholderLayout from "@/components/layout/PlaceholderLayout";
import { Code } from "@/components/ui/code";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { ArrowRight, ChevronRight } from "lucide-react";
import { Link } from "react-router-dom";

const ApiDocs = () => {
  useEffect(() => {
    // Scroll to the top of the page when component mounts
    window.scrollTo(0, 0);
  }, []);
  
  const [activeTab, setActiveTab] = useState("rest");
  
  return (
    <PlaceholderLayout title="API Documentation">
      <div className="max-w-4xl mx-auto">
        <div className="text-center mb-8">
          <p className="text-muted-foreground mb-6">
            Our API provides multiple integration options for your robotics applications.
          </p>
        </div>
        
        <Tabs defaultValue="rest" value={activeTab} onValueChange={setActiveTab} className="mb-10">
          <TabsList className="mb-6 grid w-full grid-cols-2 h-auto">
            <TabsTrigger value="rest">REST API</TabsTrigger>
            <TabsTrigger value="websocket">WebSocket API</TabsTrigger>
          </TabsList>
          
          <TabsContent value="rest">
            <div className="space-y-8">
              <Card>
                <CardHeader>
                  <CardTitle>REST API</CardTitle>
                  <CardDescription>
                    Send telemetry data using standard HTTP requests
                  </CardDescription>
                </CardHeader>
                <CardContent>
                  <h3 className="text-lg font-medium mb-3">Endpoint</h3>
                  <p className="mb-4">
                    <code className="bg-muted p-1 rounded">POST https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry</code>
                  </p>
                  
                  <h3 className="text-lg font-medium mb-3 mt-6">Authentication</h3>
                  <p className="mb-2">
                    Add your API key to the request headers:
                  </p>
                  <Code>
                    {`
{
  "Content-Type": "application/json",
  "api-key": "YOUR_API_KEY"
}
                  `}
                  </Code>
                  
                  <div className="my-6">
                    <h3 className="text-lg font-medium mb-3">Example Request</h3>
                    <Code>
                      {`
// Example API call
const response = await fetch('https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
    'api-key': 'YOUR_API_KEY'
  },
  body: JSON.stringify({
    robotId: 'YOUR_ROBOT_ID',
    batteryLevel: 75,
    temperature: 28.5,
    status: 'OK',
    location: {
      latitude: 37.7749,
      longitude: -122.4194
    },
    customTelemetry: {
      motorSpeed: 1200,
      errorCount: 0
    }
  })
});

const result = await response.json();
                      `}
                    </Code>
                  </div>
                  
                  <div className="mb-4">
                    <h3 className="text-lg font-medium mb-3">Response</h3>
                    <Code>
                      {`
{
  "success": true,
  "message": "Telemetry data received",
  "alertsProcessed": true
}
                      `}
                    </Code>
                  </div>
                </CardContent>
              </Card>
              
              <Card>
                <CardHeader>
                  <CardTitle>Retrieve Telemetry Data</CardTitle>
                  <CardDescription>
                    Fetch historical telemetry data for your robots
                  </CardDescription>
                </CardHeader>
                <CardContent>
                  <h3 className="text-lg font-medium mb-3">Endpoint</h3>
                  <p className="mb-4">
                    <code className="bg-muted p-1 rounded">GET https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/get-telemetry</code>
                  </p>
                  
                  <p className="mb-4 text-sm text-muted-foreground">
                    Returns telemetry history for the specified robot within your retention period.
                  </p>
                  
                  <div className="my-6">
                    <h3 className="text-lg font-medium mb-3">Example Request</h3>
                    <Code>
                      {`
// Example API call
const response = await fetch('https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/get-telemetry?robotId=YOUR_ROBOT_ID&limit=100&page=1', {
  method: 'GET',
  headers: {
    'api-key': 'YOUR_API_KEY'
  }
});

const data = await response.json();
                      `}
                    </Code>
                  </div>
                </CardContent>
              </Card>
            </div>
          </TabsContent>
          
          <TabsContent value="websocket">
            <div className="space-y-8">
              <Card>
                <CardHeader>
                  <CardTitle>WebSocket API</CardTitle>
                  <CardDescription>
                    Stream telemetry data in real-time using WebSockets
                  </CardDescription>
                </CardHeader>
                <CardContent>
                  <h3 className="text-lg font-medium mb-3">WebSocket Endpoint</h3>
                  <p className="mb-4">
                    <code className="bg-muted p-1 rounded">wss://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/websocket-telemetry</code>
                  </p>
                  
                  <h3 className="text-lg font-medium mb-3 mt-6">Connection Parameters</h3>
                  <p className="mb-2">
                    Connect with your API key and robot ID as URL parameters:
                  </p>
                  <Code>
                    {`
wss://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/websocket-telemetry?robotId=YOUR_ROBOT_ID&api-key=YOUR_API_KEY
                  `}
                  </Code>
                  
                  <div className="my-6">
                    <h3 className="text-lg font-medium mb-3">Sending Telemetry Data</h3>
                    <p className="mb-2">
                      Send JSON formatted telemetry data through the WebSocket connection:
                    </p>
                    <Code>
                      {`
// Example WebSocket message
ws.send(JSON.stringify({
  batteryLevel: 85,
  temperature: 27.5,
  status: "OK",
  location: {
    latitude: 37.7749,
    longitude: -122.4194
  },
  customTelemetry: {
    motorSpeed: 1200,
    errorCount: 0,
    armPosition: "extended"
  },
  timestamp: new Date().toISOString()
}));
                      `}
                    </Code>
                  </div>
                  
                  <div className="mb-4">
                    <h3 className="text-lg font-medium mb-3">Server Responses</h3>
                    <p className="mb-2">
                      The server will send JSON responses for each telemetry message received:
                    </p>
                    <Code>
                      {`
{
  "success": true,
  "message": "Telemetry data received",
  "timestamp": "2025-05-07T20:25:12.345Z"
}
                      `}
                    </Code>
                  </div>
                  
                  <div className="mt-6 p-4 bg-amber-50 border-l-4 border-amber-500 dark:bg-amber-950/20 dark:border-amber-500/50 rounded">
                    <h4 className="font-medium text-amber-800 dark:text-amber-400 mb-2">WebSocket Benefits</h4>
                    <ul className="list-disc pl-5 space-y-1 text-sm text-amber-700 dark:text-amber-300">
                      <li>Lower latency for real-time applications</li>
                      <li>Reduced overhead compared to polling with HTTP</li>
                      <li>Ideal for high-frequency telemetry updates</li>
                      <li>Supports bidirectional communication</li>
                      <li>Better state management for large-scale deployments</li>
                    </ul>
                  </div>
                </CardContent>
              </Card>
              
              <Card>
                <CardHeader>
                  <CardTitle>WebSocket Example</CardTitle>
                  <CardDescription>
                    JavaScript example for WebSocket connection
                  </CardDescription>
                </CardHeader>
                <CardContent>
                  <Code>
                    {`
// Connect to WebSocket
const apiKey = 'YOUR_API_KEY';
const robotId = 'YOUR_ROBOT_ID';
const ws = new WebSocket(\`wss://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/websocket-telemetry?robotId=\${robotId}&api-key=\${apiKey}\`);

// Connection established
ws.onopen = () => {
  console.log('WebSocket connection established');
  
  // Send telemetry data periodically
  setInterval(() => {
    const telemetryData = {
      batteryLevel: 85,
      temperature: 27.5,
      status: "OK",
      location: {
        latitude: 37.7749,
        longitude: -122.4194
      },
      customTelemetry: {
        motorSpeed: 1200,
        errorCount: 0,
        armPosition: "extended"
      },
      timestamp: new Date().toISOString()
    };
    
    ws.send(JSON.stringify(telemetryData));
  }, 5000); // Send every 5 seconds
};

// Handle server responses
ws.onmessage = (event) => {
  const response = JSON.parse(event.data);
  console.log('Server response:', response);
};

// Handle errors and connection close
ws.onerror = (error) => {
  console.error('WebSocket error:', error);
};

ws.onclose = () => {
  console.log('WebSocket connection closed');
};
                    `}
                  </Code>
                </CardContent>
              </Card>
            </div>
          </TabsContent>
        </Tabs>
        
        <div className="mt-12 p-6 border border-dashed rounded-lg text-center">
          <h2 className="text-xl font-semibold mb-2">Need More Information?</h2>
          <p className="text-muted-foreground mb-6">
            Check out our integration guide for more examples and detailed documentation.
          </p>
          <Link 
            to="/integration" 
            className="inline-flex items-center text-primary hover:underline"
          >
            View Integration Guide
            <ArrowRight className="ml-2 h-4 w-4" />
          </Link>
        </div>
      </div>
    </PlaceholderLayout>
  );
};

export default ApiDocs;
