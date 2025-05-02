
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { useState } from "react";
import { toast } from "sonner";
import { ClipboardCopy, AlertTriangle } from "lucide-react";

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
    "api-key": "YOUR_API_KEY" // Get this from API Key panel in sidebar
  },
  body: JSON.stringify({
    robotId: "YOUR_ROBOT_ID", // Get this from your robot details
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
    "api-key": "YOUR_API_KEY"  # Get this from API Key panel in sidebar
}
data = {
    "robotId": "YOUR_ROBOT_ID",  # Get this from your robot details
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
          Send robot telemetry data using these example code snippets. Replace <code>YOUR_ROBOT_ID</code> with your robot's ID and <code>YOUR_API_KEY</code> with your API key from the "API Key" panel in the sidebar.
        </CardDescription>
      </CardHeader>
      <CardContent>
        <div className="mb-6 p-4 border border-red-200 bg-red-50 dark:border-red-900 dark:bg-red-950/30 rounded-md">
          <div className="flex gap-3">
            <AlertTriangle className="h-5 w-5 text-red-600 dark:text-red-400 mt-0.5" />
            <div>
              <h4 className="font-medium text-red-800 dark:text-red-300 mb-2">IMPORTANT: API Key Format</h4>
              <p className="text-sm text-red-700 dark:text-red-400">
                The API key <strong>must</strong> be sent in the <code className="bg-red-100 dark:bg-red-900 px-1.5 py-0.5 rounded text-xs">api-key</code> header (case sensitive). 
                Using other formats like <code>Authorization</code> will not work.
              </p>
            </div>
          </div>
        </div>

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
        
        <div className="mt-6 p-4 border border-amber-200 bg-amber-50 dark:border-amber-900 dark:bg-amber-950/30 rounded-md">
          <h4 className="font-medium text-amber-800 dark:text-amber-300 mb-2">Troubleshooting</h4>
          <ul className="list-disc pl-5 text-sm text-amber-700 dark:text-amber-400 space-y-1">
            <li>Make sure to replace <code>YOUR_API_KEY</code> with the API key from the API Key panel in the sidebar</li>
            <li>The API key must be sent in the <code>api-key</code> header (case sensitive)</li>
            <li>Your robot must exist in your account and the Robot ID must be correct</li>
            <li>For debugging, add more verbose logging to your requests and check response status codes</li>
            <li>If you're still having issues, try downloading and running the robot-client.js simulator</li>
          </ul>
        </div>
      </CardContent>
    </Card>
  );
}
