
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { useState } from "react";
import { toast } from "@/components/ui/sonner";
import { ClipboardCopy, Play, AlertCircle } from "lucide-react";

export function WebSocketExample() {
  const [activeTab, setActiveTab] = useState("javascript");
  const [isLiveDemo, setIsLiveDemo] = useState(false);
  const [demoStatus, setDemoStatus] = useState<string | null>(null);
  const [demoLogs, setDemoLogs] = useState<string[]>([]);
  const [hasError, setHasError] = useState(false);

  const javascriptExample = `// Browser WebSocket Example
const apiKey = 'YOUR_API_KEY';
const robotId = 'YOUR_ROBOT_ID';

// Connect to the WebSocket server
const wsUrl = 'wss://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/websocket-telemetry?robotId=' + robotId + '&api-key=' + apiKey;
const ws = new WebSocket(wsUrl);

ws.onopen = () => {
  console.log('WebSocket connection established');
  
  // Send telemetry data
  setInterval(() => {
    const telemetryData = {
      batteryLevel: Math.floor(Math.random() * 100),
      temperature: 25 + (Math.random() * 10),
      status: "OK",
      location: {
        latitude: 37.7749,
        longitude: -122.4194 + (Math.random() * 0.01)
      },
      customTelemetry: {
        motorSpeed: 1200 + Math.floor(Math.random() * 100),
        errorCount: 0,
        armPosition: "extended"
      },
      timestamp: new Date().toISOString()
    };
    
    ws.send(JSON.stringify(telemetryData));
  }, 5000); // Send every 5 seconds
};

ws.onmessage = (event) => {
  const response = JSON.parse(event.data);
  console.log('Received response:', response);
  
  // Check for errors or warnings
  if (response.error) {
    console.error('Error from server:', response.error, response.details);
  }
  
  if (response.warning) {
    console.warn('Warning from server:', response.warning, response.details);
  }
};

ws.onerror = (error) => {
  console.error('WebSocket error:', error);
};

ws.onclose = () => {
  console.log('WebSocket connection closed');
};

// Close the connection when done
// ws.close();`;

  const pythonExample = `# Python WebSocket Example
import asyncio
import websockets
import json
import random
import time
from datetime import datetime

API_KEY = "YOUR_API_KEY"
ROBOT_ID = "YOUR_ROBOT_ID"

async def send_telemetry():
    # Connect to WebSocket server
    ws_url = f"wss://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/websocket-telemetry?robotId={ROBOT_ID}&api-key={API_KEY}"
    
    try:
        async with websockets.connect(ws_url) as websocket:
            print("WebSocket connection established")
            
            # Send telemetry data every 5 seconds
            while True:
                telemetry_data = {
                    "batteryLevel": random.randint(50, 100),
                    "temperature": 25 + (random.random() * 10),
                    "status": "OK",
                    "location": {
                        "latitude": 37.7749,
                        "longitude": -122.4194 + (random.random() * 0.01)
                    },
                    "customTelemetry": {
                        "motorSpeed": 1200 + random.randint(0, 100),
                        "errorCount": 0,
                        "armPosition": "extended"
                    },
                    "timestamp": datetime.utcnow().isoformat()
                }
                
                await websocket.send(json.dumps(telemetry_data))
                print(f"Sent telemetry data: {telemetry_data}")
                
                # Wait for response
                response = await websocket.recv()
                response_data = json.loads(response)
                print(f"Received response: {response}")
                
                # Handle errors or warnings
                if "error" in response_data:
                    print(f"ERROR: {response_data['error']} - {response_data.get('details', '')}")
                
                if "warning" in response_data:
                    print(f"WARNING: {response_data['warning']} - {response_data.get('details', '')}")
                
                await asyncio.sleep(5)
    except Exception as e:
        print(f"Connection error: {str(e)}")

asyncio.run(send_telemetry())`;

  const nodeJsExample = `// Node.js WebSocket Example
const WebSocket = require('ws');

const API_KEY = 'YOUR_API_KEY';
const ROBOT_ID = 'YOUR_ROBOT_ID';

// Connect to the WebSocket server
const wsUrl = \`wss://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/websocket-telemetry?robotId=\${ROBOT_ID}&api-key=\${API_KEY}\`;
const ws = new WebSocket(wsUrl);

ws.on('open', () => {
  console.log('WebSocket connection established');
  
  // Send telemetry data every 5 seconds
  setInterval(() => {
    const telemetryData = {
      batteryLevel: Math.floor(Math.random() * 100),
      temperature: 25 + (Math.random() * 10),
      status: "OK",
      location: {
        latitude: 37.7749,
        longitude: -122.4194 + (Math.random() * 0.01)
      },
      customTelemetry: {
        motorSpeed: 1200 + Math.floor(Math.random() * 100),
        errorCount: 0,
        armPosition: "extended"
      },
      timestamp: new Date().toISOString()
    };
    
    ws.send(JSON.stringify(telemetryData));
    console.log('Sent telemetry data:', telemetryData);
  }, 5000);
});

ws.on('message', (data) => {
  const response = JSON.parse(data);
  console.log('Received response:', response);
  
  // Handle errors or warnings
  if (response.error) {
    console.error('Error from server:', response.error);
    console.error('Details:', response.details || 'No details provided');
  }
  
  if (response.warning) {
    console.warn('Warning from server:', response.warning);
    console.warn('Details:', response.details || 'No details provided');
  }
});

ws.on('error', (error) => {
  console.error('WebSocket error:', error);
});

ws.on('close', (code, reason) => {
  console.log(\`WebSocket connection closed: \${code} - \${reason || 'No reason provided'}\`);
});`;

  const copyToClipboard = (text: string) => {
    navigator.clipboard.writeText(text);
    toast("Code copied", {
      description: "Example code has been copied to your clipboard"
    });
  };

  const startLiveDemo = () => {
    setIsLiveDemo(true);
    setDemoLogs(["Starting WebSocket connection simulation..."]);
    setHasError(false);
    
    // Simulate WebSocket connection
    setTimeout(() => {
      setDemoStatus("connected");
      setDemoLogs(prev => [...prev, "WebSocket connection established"]);
      
      // Simulate sending data
      let counter = 0;
      const interval = setInterval(() => {
        counter++;
        const batteryLevel = Math.floor(Math.random() * 100);
        const temperature = (25 + (Math.random() * 10)).toFixed(1);
        
        setDemoLogs(prev => [
          ...prev, 
          `Sending telemetry packet #${counter}...`,
          `Data: Battery ${batteryLevel}%, Temperature ${temperature}°C`
        ]);

        setTimeout(() => {
          if (counter === 2) {
            // Simulate an error on the second packet for demonstration
            setHasError(true);
            setDemoLogs(prev => [
              ...prev, 
              "ERROR: Authentication failed - Details: The provided API key is invalid or has been revoked."
            ]);
            
            toast({
              title: "Connection error",
              description: "Authentication failed. Invalid API key.",
              variant: "destructive"
            });

            clearInterval(interval);
            setDemoLogs(prev => [...prev, "Connection terminated due to authentication error.", "Please check your API key and try again."]);
            setDemoStatus("error");
          } else {
            setDemoLogs(prev => [...prev, "Server response: Data received successfully"]);
          }
        }, 500);
        
        if (counter >= 3 && !hasError) {
          clearInterval(interval);
          setDemoLogs(prev => [...prev, "Demo completed. In a real implementation, this connection would stay open."]);
        }
      }, 2000);
    }, 1000);
  };

  const stopLiveDemo = () => {
    setIsLiveDemo(false);
    setDemoStatus(null);
    setDemoLogs([]);
    setHasError(false);
  };

  // Example of common errors section
  const commonErrors = [
    {
      error: "Authentication failed",
      details: "The provided API key is invalid or has been revoked.",
      solution: "Check that your API key is correct and still active in your account settings."
    },
    {
      error: "Access denied",
      details: "Invalid robot ID or you don't have access to this robot.",
      solution: "Verify the robot ID is correct and belongs to your account."
    },
    {
      error: "Missing parameter",
      details: "Robot ID is required.",
      solution: "Include the robotId parameter in your WebSocket connection URL."
    }
  ];

  return (
    <Card>
      <CardHeader>
        <CardTitle>Real-time Telemetry with WebSockets</CardTitle>
        <CardDescription>
          For more frequent updates or real-time monitoring, use our WebSocket API for streaming telemetry data.
          This approach is recommended for applications requiring continuous updates or high-frequency telemetry.
        </CardDescription>
      </CardHeader>
      <CardContent>
        <Tabs defaultValue="javascript" value={activeTab} onValueChange={setActiveTab}>
          <TabsList className="mb-4">
            <TabsTrigger value="javascript">Browser JavaScript</TabsTrigger>
            <TabsTrigger value="nodejs">Node.js</TabsTrigger>
            <TabsTrigger value="python">Python</TabsTrigger>
          </TabsList>
          
          <TabsContent value="javascript" className="relative">
            <Button 
              variant="ghost" 
              size="icon"
              className="absolute top-2 right-2"
              onClick={() => copyToClipboard(javascriptExample)}
            >
              <ClipboardCopy size={16} />
              <span className="sr-only">Copy code</span>
            </Button>
            <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
              {javascriptExample}
            </pre>
          </TabsContent>
          
          <TabsContent value="nodejs" className="relative">
            <Button 
              variant="ghost" 
              size="icon"
              className="absolute top-2 right-2"
              onClick={() => copyToClipboard(nodeJsExample)}
            >
              <ClipboardCopy size={16} />
              <span className="sr-only">Copy code</span>
            </Button>
            <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
              {nodeJsExample}
            </pre>
          </TabsContent>
          
          <TabsContent value="python" className="relative">
            <Button 
              variant="ghost" 
              size="icon"
              className="absolute top-2 right-2"
              onClick={() => copyToClipboard(pythonExample)}
            >
              <ClipboardCopy size={16} />
              <span className="sr-only">Copy code</span>
            </Button>
            <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
              {pythonExample}
            </pre>
          </TabsContent>
        </Tabs>
        
        {/* Live Demo Card */}
        <div className="mt-6 border rounded-lg p-4">
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-medium">WebSocket Demo</h3>
            {!isLiveDemo ? (
              <Button size="sm" onClick={startLiveDemo}>
                <Play size={16} className="mr-2" />
                Start Demo
              </Button>
            ) : (
              <Button size="sm" variant="outline" onClick={stopLiveDemo}>
                Stop Demo
              </Button>
            )}
          </div>
          
          {demoStatus && (
            <div className="flex items-center gap-2 mb-2">
              <div className={`w-3 h-3 rounded-full ${
                demoStatus === 'connected' ? 'bg-green-500' : 
                demoStatus === 'error' ? 'bg-red-500' : 'bg-amber-500'
              }`}></div>
              <span className="text-sm font-medium">
                {demoStatus === 'connected' ? 'Connected' : 
                 demoStatus === 'error' ? 'Connection Error' : 'Connecting...'}
              </span>
            </div>
          )}
          
          <div className="bg-black/90 text-green-400 p-3 rounded font-mono text-xs h-40 overflow-y-auto">
            {demoLogs.length > 0 ? (
              demoLogs.map((log, index) => (
                <div key={index} className={`mb-1 ${log.includes("ERROR:") ? "text-red-400" : ""}`}>
                  {`> ${log}`}
                </div>
              ))
            ) : (
              <div className="text-gray-500 italic">Click "Start Demo" to simulate WebSocket communication</div>
            )}
          </div>
        </div>
        
        {/* Common Errors Section */}
        <div className="mt-6 border rounded-lg p-4">
          <div className="flex items-center gap-2 mb-4">
            <AlertCircle size={18} className="text-amber-500" />
            <h3 className="text-lg font-medium">Common Error Responses</h3>
          </div>
          
          <div className="space-y-4">
            {commonErrors.map((error, index) => (
              <div key={index} className="bg-muted/50 p-3 rounded">
                <div className="font-medium text-red-500">{error.error}</div>
                <div className="text-sm text-muted-foreground mt-1">{error.details}</div>
                <div className="text-sm mt-1"><span className="font-medium">Solution:</span> {error.solution}</div>
              </div>
            ))}
          </div>
        </div>
      </CardContent>
      <CardFooter className="flex flex-col items-start text-sm text-muted-foreground">
        <p className="mb-2">
          WebSocket connections maintain an open connection for bidirectional, real-time data transfer.
        </p>
        <p className="mb-2">
          This is ideal for robotics applications requiring continuous telemetry streams or low-latency communication.
        </p>
        <p>
          <strong>Error Handling:</strong> The WebSocket API provides detailed error messages to help you troubleshoot connection issues.
          Check the response for error and details fields when a message is received.
        </p>
      </CardFooter>
    </Card>
  );
}
