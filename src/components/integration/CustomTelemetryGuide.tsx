
import { UserProfile } from "@/types/robot";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Code } from "@/components/ui/code";

interface CustomTelemetryGuideProps {
  apiKey: string | null;
  robotId: string;
  customTelemetryTypes: string[] | null;
}

export function CustomTelemetryGuide({ apiKey, robotId, customTelemetryTypes }: CustomTelemetryGuideProps) {
  const sampleCustomData = () => {
    const customData: Record<string, any> = {};
    
    // Add examples of custom telemetry based on user's defined types
    if (customTelemetryTypes && customTelemetryTypes.length > 0) {
      customTelemetryTypes.forEach((type, index) => {
        // Generate sample data based on the type name
        if (type.toLowerCase().includes("temp")) {
          customData[type] = 22.5;
        } else if (type.toLowerCase().includes("error")) {
          customData[type] = 0;
        } else if (type.toLowerCase().includes("status")) {
          customData[type] = "operational";
        } else if (type.toLowerCase().includes("level")) {
          customData[type] = 75;
        } else {
          // Default sample values
          customData[type] = index % 2 === 0 ? 42 : "OK";
        }
      });
    } else {
      // Default examples if no custom types are defined
      customData.motorSpeed = 1200;
      customData.armPosition = "extended";
      customData.errorCount = 0;
    }
    
    return customData;
  };

  const fullExample = {
    robotId: robotId,
    batteryLevel: 85,
    temperature: 27.5,
    status: "OK",
    location: {
      latitude: 37.7749,
      longitude: -122.4194
    },
    timestamp: new Date().toISOString(),
    customTelemetry: sampleCustomData()
  };

  const curlCommand = `curl -X POST "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry" \\
  -H "Content-Type: application/json" \\
  -H "api-key: ${apiKey || 'YOUR_API_KEY'}" \\
  -d '${JSON.stringify(fullExample, null, 2)}'`;

  const pythonCode = `import requests
import json
from datetime import datetime

url = "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry"

headers = {
    "Content-Type": "application/json",
    "api-key": "${apiKey || 'YOUR_API_KEY'}"
}

data = {
    "robotId": "${robotId}",
    "batteryLevel": 85,
    "temperature": 27.5,
    "status": "OK",
    "location": {
        "latitude": 37.7749,
        "longitude": -122.4194
    },
    "timestamp": datetime.now().isoformat(),
    "customTelemetry": ${JSON.stringify(sampleCustomData(), null, 4).replace(/"/g, "'")}
}

response = requests.post(url, headers=headers, json=data)
print(response.status_code)
print(response.json())`;

  const javascriptCode = `// JavaScript/Node.js example
const axios = require('axios');

const sendTelemetry = async () => {
  try {
    const response = await axios.post(
      'https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry',
      {
        robotId: '${robotId}',
        batteryLevel: 85,
        temperature: 27.5,
        status: 'OK',
        location: {
          latitude: 37.7749,
          longitude: -122.4194
        },
        timestamp: new Date().toISOString(),
        customTelemetry: ${JSON.stringify(sampleCustomData(), null, 2)}
      },
      {
        headers: {
          'Content-Type': 'application/json',
          'api-key': '${apiKey || 'YOUR_API_KEY'}'
        }
      }
    );
    
    console.log('Response:', response.data);
  } catch (error) {
    console.error('Error sending telemetry:', error);
  }
};

sendTelemetry();`;

  return (
    <Card className="mb-4">
      <CardHeader>
        <CardTitle>Send Custom Telemetry</CardTitle>
        <CardDescription>
          Use these examples to send custom telemetry data for your robot
        </CardDescription>
      </CardHeader>
      <CardContent>
        <Tabs defaultValue="curl">
          <TabsList>
            <TabsTrigger value="curl">cURL</TabsTrigger>
            <TabsTrigger value="python">Python</TabsTrigger>
            <TabsTrigger value="javascript">JavaScript</TabsTrigger>
          </TabsList>
          <TabsContent value="curl">
            <div className="bg-muted p-4 rounded-md overflow-x-auto">
              <pre className="text-xs">{curlCommand}</pre>
            </div>
          </TabsContent>
          <TabsContent value="python">
            <div className="bg-muted p-4 rounded-md overflow-x-auto">
              <pre className="text-xs">{pythonCode}</pre>
            </div>
          </TabsContent>
          <TabsContent value="javascript">
            <div className="bg-muted p-4 rounded-md overflow-x-auto">
              <pre className="text-xs">{javascriptCode}</pre>
            </div>
          </TabsContent>
        </Tabs>
      </CardContent>
      <CardFooter className="flex flex-col items-start">
        <p className="text-sm text-muted-foreground mb-2">
          <strong>Note:</strong> Your API key is shown above. Keep it secret!
        </p>
        <p className="text-sm text-muted-foreground">
          You can define custom telemetry types in your profile settings.
        </p>
      </CardFooter>
    </Card>
  );
}
