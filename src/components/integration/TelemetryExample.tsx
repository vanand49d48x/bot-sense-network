
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { useState } from "react";
import { toast } from "@/components/ui/sonner";
import { ClipboardCopy } from "lucide-react";

export function TelemetryExample() {
  const [activeTab, setActiveTab] = useState("curl");

  const curlExample = `curl -X POST "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry" \\
  -H "Content-Type: application/json" \\
  -H "api-key: YOUR_API_KEY" \\
  -d '{
    "robotId": "YOUR_ROBOT_ID",
    "batteryLevel": 75,
    "temperature": 28.5,
    "status": "OK",
    "location": {
      "latitude": 37.7749,
      "longitude": -122.4194
    },
    "timestamp": "'"$(date -u +"%Y-%m-%dT%H:%M:%SZ")"'"
  }'`;

  const javascriptExample = `// Using fetch API
fetch("https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry", {
  method: "POST",
  headers: {
    "Content-Type": "application/json",
    "api-key": "YOUR_API_KEY"
  },
  body: JSON.stringify({
    robotId: "YOUR_ROBOT_ID",
    batteryLevel: 75,
    temperature: 28.5,
    status: "OK",
    location: {
      latitude: 37.7749,
      longitude: -122.4194
    },
    timestamp: new Date().toISOString()
  })
})
.then(response => response.json())
.then(data => console.log(data))
.catch(error => console.error("Error:", error));`;

  const pythonExample = `import requests
import json
from datetime import datetime

url = "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry"
headers = {
    "Content-Type": "application/json",
    "api-key": "YOUR_API_KEY"
}
data = {
    "robotId": "YOUR_ROBOT_ID",
    "batteryLevel": 75,
    "temperature": 28.5,
    "status": "OK",
    "location": {
        "latitude": 37.7749,
        "longitude": -122.4194
    },
    "timestamp": datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")
}

response = requests.post(url, headers=headers, data=json.dumps(data))
print(response.text)`;

  const copyToClipboard = (text: string) => {
    navigator.clipboard.writeText(text);
    toast("Code copied", {
      description: "Example code has been copied to your clipboard"
    });
  };

  return (
    <Card>
      <CardHeader>
        <CardTitle>Sending Telemetry Data</CardTitle>
        <CardDescription>
          Send robot telemetry data using these example code snippets. Replace <code>YOUR_ROBOT_ID</code> with your robot's UUID and <code>YOUR_API_KEY</code> with your central API key from the API Integration section in the sidebar.
        </CardDescription>
      </CardHeader>
      <CardContent>
        <Tabs defaultValue="curl" value={activeTab} onValueChange={setActiveTab}>
          <TabsList className="mb-4">
            <TabsTrigger value="curl">cURL</TabsTrigger>
            <TabsTrigger value="javascript">JavaScript</TabsTrigger>
            <TabsTrigger value="python">Python</TabsTrigger>
          </TabsList>
          <TabsContent value="curl" className="relative">
            <Button 
              variant="ghost" 
              size="icon"
              className="absolute top-2 right-2"
              onClick={() => copyToClipboard(curlExample)}
            >
              <ClipboardCopy size={16} />
              <span className="sr-only">Copy code</span>
            </Button>
            <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
              {curlExample}
            </pre>
          </TabsContent>
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
      </CardContent>
    </Card>
  );
}
