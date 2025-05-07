import { useState } from "react";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Code } from "@/components/ui/code";
import { Alert, AlertDescription, AlertTitle } from "@/components/ui/alert";
import { InfoIcon } from "lucide-react";
import { useAuth } from "@/context/AuthContext";
import { useQuery } from "@tanstack/react-query";
import { supabase } from "@/integrations/supabase/client";
import { UserProfile } from "@/types/robot";
import { Link } from "react-router-dom";

export function CustomTelemetryExample() {
  const [activeTab, setActiveTab] = useState("overview");
  const { user } = useAuth();

  // Fetch user's profile to get API key and custom telemetry types
  const { data: profile } = useQuery({
    queryKey: ['profile', user?.id],
    queryFn: async () => {
      if (!user?.id) return null;
      
      const { data, error } = await supabase
        .from('profiles')
        .select('id, api_key, custom_telemetry_types')
        .eq('id', user.id)
        .single();
        
      if (error) {
        console.error('Error fetching profile:', error);
        return null;
      }
      
      return data as UserProfile;
    },
    enabled: !!user?.id,
  });

  // Fetch user's robots to provide an example robot ID
  const { data: robots } = useQuery({
    queryKey: ['robots', user?.id],
    queryFn: async () => {
      if (!user?.id) return [];
      
      const { data, error } = await supabase
        .from('robots')
        .select('id, name')
        .eq('user_id', user.id);
        
      if (error) {
        console.error('Error fetching robots:', error);
        return [];
      }
      
      return data;
    },
    enabled: !!user?.id,
  });

  const exampleRobotId = robots && robots.length > 0 ? robots[0].id : 'YOUR_ROBOT_ID';
  const apiKey = profile?.api_key || 'YOUR_API_KEY';
  const customTypes = profile?.custom_telemetry_types || [];

  const createSampleData = () => {
    const customData: Record<string, any> = {};
    
    if (customTypes.length > 0) {
      // Generate example data based on user-defined custom types
      customTypes.forEach((type, index) => {
        if (typeof type === 'string') {
          if (type.toLowerCase().includes('temp')) {
            customData[type] = 23.5;
          } else if (type.toLowerCase().includes('speed') || type.toLowerCase().includes('rpm')) {
            customData[type] = 1200;
          } else if (type.toLowerCase().includes('status')) {
            customData[type] = 'operational';
          } else if (type.toLowerCase().includes('level') || type.toLowerCase().includes('percentage')) {
            customData[type] = 75;
          } else if (type.toLowerCase().includes('count') || type.toLowerCase().includes('error')) {
            customData[type] = 0;
          } else if (type.toLowerCase().includes('position')) {
            customData[type] = 'extended';
          } else {
            // Default case based on index
            customData[type] = index % 2 === 0 ? 42 : 'normal';
          }
        }
      });
    } else {
      // Default examples if no custom types defined
      customData.motorSpeed = 1200;
      customData.armPosition = 'extended';
      customData.errorCount = 0;
    }
    
    return customData;
  };

  const examplePayload = {
    robotId: exampleRobotId,
    batteryLevel: 85,
    temperature: 27.5,
    status: "OK",
    location: {
      latitude: 37.7749,
      longitude: -122.4194
    },
    customTelemetry: createSampleData()
  };

  const curlExample = `curl -X POST "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry" \\
  -H "Content-Type: application/json" \\
  -H "api-key: ${apiKey}" \\
  -d '${JSON.stringify(examplePayload, null, 2)}'`;

  const pythonExample = `import requests
import json

url = "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry"

headers = {
    "Content-Type": "application/json",
    "api-key": "${apiKey}"
}

data = ${JSON.stringify(examplePayload, null, 4)}

response = requests.post(url, headers=headers, json=data)
print(response.status_code)
print(response.json())`;

  const javascriptExample = `// JavaScript/Node.js example
const axios = require('axios');

const sendTelemetry = async () => {
  try {
    const response = await axios.post(
      'https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry',
      ${JSON.stringify(examplePayload, null, 6)},
      {
        headers: {
          'Content-Type': 'application/json',
          'api-key': '${apiKey}'
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
    <>
      <section className="mb-8">
        <h2 className="text-2xl font-semibold mb-4">Custom Telemetry</h2>
        <p className="text-muted-foreground mb-6 max-w-3xl">
          Send custom telemetry data specific to your robot types. Define your own telemetry fields and send them through our API.
        </p>
      </section>

      <Tabs defaultValue={activeTab} onValueChange={setActiveTab}>
        <TabsList className="mb-6">
          <TabsTrigger value="overview">Overview</TabsTrigger>
          <TabsTrigger value="examples">Code Examples</TabsTrigger>
          <TabsTrigger value="setup">Setup Guide</TabsTrigger>
        </TabsList>
        
        <TabsContent value="overview">
          <div className="space-y-6">
            <Card>
              <CardHeader>
                <CardTitle>Custom Telemetry Overview</CardTitle>
                <CardDescription>
                  Send any custom data from your robots to the platform
                </CardDescription>
              </CardHeader>
              <CardContent className="space-y-4">
                <p>
                  Our platform allows you to define and send custom telemetry data tailored to your specific robot types.
                  This feature enables you to track metrics beyond the standard battery, temperature and location data.
                </p>
                
                <h3 className="text-lg font-medium mt-4">Supported Features</h3>
                <ul className="list-disc pl-6 space-y-2">
                  <li>Define your own custom telemetry types in your profile</li>
                  <li>Send any numeric or text values for your custom metrics</li>
                  <li>Set up alerts based on custom telemetry thresholds</li>
                  <li>View custom telemetry in real-time on the dashboard</li>
                  <li>Historical tracking of all custom metrics</li>
                </ul>
              </CardContent>
            </Card>
            
            <Alert>
              <InfoIcon className="h-4 w-4" />
              <AlertTitle>Get Started</AlertTitle>
              <AlertDescription>
                First, define your custom telemetry types in your <Link to="/profile" className="underline text-primary">profile settings</Link>, 
                then use the examples below to send custom data.
              </AlertDescription>
            </Alert>
            
            <Card>
              <CardHeader>
                <CardTitle>How Custom Telemetry Works</CardTitle>
              </CardHeader>
              <CardContent>
                <ol className="list-decimal pl-6 space-y-4">
                  <li>
                    <strong>Define custom types</strong>
                    <p className="text-muted-foreground mt-1">
                      Go to your profile settings and define the types of custom telemetry you want to send 
                      (e.g., "motorSpeed", "armPosition", "errorCount").
                    </p>
                  </li>
                  <li>
                    <strong>Send telemetry data</strong>
                    <p className="text-muted-foreground mt-1">
                      Include a "customTelemetry" object in your API requests with keys matching your defined types.
                    </p>
                  </li>
                  <li>
                    <strong>View in dashboard</strong>
                    <p className="text-muted-foreground mt-1">
                      Your custom telemetry will appear in the robot detail view under the "Custom Telemetry" tab.
                    </p>
                  </li>
                  <li>
                    <strong>Set up alerts</strong>
                    <p className="text-muted-foreground mt-1">
                      Configure alerts based on thresholds for your custom metrics in the alerts settings.
                    </p>
                  </li>
                </ol>
              </CardContent>
            </Card>
          </div>
        </TabsContent>
        
        <TabsContent value="examples">
          <Card>
            <CardHeader>
              <CardTitle>Custom Telemetry Code Examples</CardTitle>
              <CardDescription>
                Use these examples to send custom telemetry data from your robots
              </CardDescription>
            </CardHeader>
            <CardContent>
              <Tabs defaultValue="curl">
                <TabsList className="mb-4">
                  <TabsTrigger value="curl">cURL</TabsTrigger>
                  <TabsTrigger value="python">Python</TabsTrigger>
                  <TabsTrigger value="javascript">JavaScript</TabsTrigger>
                </TabsList>
                <TabsContent value="curl">
                  <Code>{curlExample}</Code>
                </TabsContent>
                <TabsContent value="python">
                  <Code>{pythonExample}</Code>
                </TabsContent>
                <TabsContent value="javascript">
                  <Code>{javascriptExample}</Code>
                </TabsContent>
              </Tabs>
              <div className="mt-4 text-sm text-muted-foreground">
                <p className="mb-2"><strong>Note:</strong> Replace placeholders with your actual robot ID and API key if not automatically filled.</p>
                <p>The <code>customTelemetry</code> object should contain keys that match the custom telemetry types you've defined in your profile settings.</p>
              </div>
            </CardContent>
            <CardFooter className="border-t pt-4">
              {!user && (
                <p className="text-sm text-amber-600">
                  Sign in to see examples customized with your API key and robot ID.
                </p>
              )}
            </CardFooter>
          </Card>
        </TabsContent>
        
        <TabsContent value="setup">
          <div className="space-y-6">
            <Card>
              <CardHeader>
                <CardTitle>Setting Up Custom Telemetry</CardTitle>
                <CardDescription>
                  Follow these steps to configure and use custom telemetry
                </CardDescription>
              </CardHeader>
              <CardContent>
                <div className="space-y-6">
                  <div className="border-b pb-4">
                    <h3 className="text-lg font-medium mb-2">Step 1: Define Custom Telemetry Types</h3>
                    <p className="mb-4">
                      Navigate to your profile settings to define the types of custom telemetry data you want to track.
                    </p>
                    <ol className="list-decimal pl-6 space-y-2">
                      <li>Go to <Link to="/profile" className="text-primary underline">Profile Settings</Link></li>
                      <li>Scroll to the "Custom Telemetry Types" section</li>
                      <li>Add your custom metrics (e.g., "motorSpeed", "errorCount")</li>
                      <li>Save your settings</li>
                    </ol>
                  </div>
                  
                  <div className="border-b pb-4">
                    <h3 className="text-lg font-medium mb-2">Step 2: Update Your Robot's Code</h3>
                    <p className="mb-2">
                      Modify your robot's code to collect and send the custom metrics you defined.
                    </p>
                    <p>
                      Use the examples in the "Code Examples" tab as a starting point. Make sure to:
                    </p>
                    <ul className="list-disc pl-6 mt-2 space-y-2">
                      <li>Include your robot's unique ID</li>
                      <li>Use your API key for authentication</li>
                      <li>Format your custom telemetry data as a nested object</li>
                    </ul>
                  </div>
                  
                  <div className="border-b pb-4">
                    <h3 className="text-lg font-medium mb-2">Step 3: Test Your Integration</h3>
                    <p>
                      After setting up your custom telemetry, send a test payload and check if it appears in your dashboard:
                    </p>
                    <ol className="list-decimal pl-6 mt-2 space-y-2">
                      <li>Send a test request using one of the code examples</li>
                      <li>Go to your <Link to="/dashboard" className="text-primary underline">Dashboard</Link></li>
                      <li>Click on your robot to view its details</li>
                      <li>Check the "Custom Telemetry" tab to verify your data is received</li>
                    </ol>
                  </div>
                  
                  <div>
                    <h3 className="text-lg font-medium mb-2">Step 4: Set Up Alerts (Optional)</h3>
                    <p>
                      Configure alerts based on custom telemetry thresholds:
                    </p>
                    <ol className="list-decimal pl-6 mt-2 space-y-2">
                      <li>Go to <Link to="/alerts" className="text-primary underline">Alert Settings</Link></li>
                      <li>Create a new alert rule</li>
                      <li>Select your custom metric as the alert type</li>
                      <li>Define thresholds and notification settings</li>
                    </ol>
                  </div>
                </div>
              </CardContent>
            </Card>
            
            <Card>
              <CardHeader>
                <CardTitle>Advanced Usage</CardTitle>
              </CardHeader>
              <CardContent>
                <div className="space-y-4">
                  <div>
                    <h3 className="font-medium mb-1">Complex Data Structures</h3>
                    <p className="text-sm text-muted-foreground">
                      You can send nested objects or arrays as custom telemetry values, but for alert processing,
                      simple numeric values work best.
                    </p>
                  </div>
                  
                  <div>
                    <h3 className="font-medium mb-1">Data Retention</h3>
                    <p className="text-sm text-muted-foreground">
                      Custom telemetry data is stored with the same retention policy as standard telemetry.
                    </p>
                  </div>
                  
                  <div>
                    <h3 className="font-medium mb-1">Filtering and Searching</h3>
                    <p className="text-sm text-muted-foreground">
                      Custom telemetry can influence robot status and be used in dashboard filtering.
                    </p>
                  </div>
                </div>
              </CardContent>
            </Card>
          </div>
        </TabsContent>
      </Tabs>
    </>
  );
}
