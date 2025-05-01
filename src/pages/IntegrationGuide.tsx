
import { Button } from "@/components/ui/button";
import { TelemetryExample } from "@/components/integration/TelemetryExample";
import { ROSIntegration } from "@/components/integration/ROSIntegration";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { AlertTriangle } from "lucide-react";

export default function IntegrationGuide() {
  return (
    <div className="container py-10">
      <div className="mb-10">
        <h1 className="text-3xl font-bold tracking-tight mb-4">Robot Integration Guide</h1>
        <p className="text-muted-foreground max-w-3xl">
          Follow these instructions to connect your robots to the platform. Each robot will need the API key for authentication when sending telemetry data.
        </p>
      </div>

      <div className="space-y-8">
        <Card className="border-amber-300 bg-amber-50 dark:bg-amber-950/30 dark:border-amber-800">
          <CardContent className="pt-6">
            <div className="flex gap-3">
              <AlertTriangle className="h-5 w-5 text-amber-600 dark:text-amber-400 mt-0.5" />
              <div>
                <h3 className="font-medium text-amber-800 dark:text-amber-300">Important Authentication Note</h3>
                <p className="text-sm text-amber-700 dark:text-amber-400 mt-1">
                  Make sure to include your API key in the <code className="bg-amber-100 dark:bg-amber-900 px-1.5 py-0.5 rounded text-xs">api-key</code> header with all telemetry requests. 
                  You can find your API key in the API Key panel in the sidebar.
                </p>
              </div>
            </div>
          </CardContent>
        </Card>
        
        <section>
          <h2 className="text-2xl font-semibold mb-4">Getting Started</h2>
          <div className="grid gap-6 lg:grid-cols-2">
            <div className="space-y-4">
              <h3 className="text-xl font-medium">1. Add your robot</h3>
              <p className="text-muted-foreground">
                First, add your robot to the dashboard. This will generate a unique identifier for your robot.
              </p>
              <Button asChild>
                <a href="/dashboard">Go to Dashboard</a>
              </Button>
            </div>
            <div className="space-y-4">
              <h3 className="text-xl font-medium">2. Find your API key</h3>
              <p className="text-muted-foreground">
                You'll need your API key for all telemetry requests. This key is shown in the API Key panel in the sidebar.
                It's a user-level key that works for all your robots.
              </p>
            </div>
          </div>
        </section>

        <section>
          <h2 className="text-2xl font-semibold mb-4">Sending Telemetry</h2>
          <p className="text-muted-foreground mb-6">
            Send telemetry data using the following endpoint: <code className="bg-muted px-1.5 py-0.5 rounded text-sm">https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry</code>
          </p>
          
          <TelemetryExample />
        </section>

        <section>
          <h2 className="text-2xl font-semibold mb-4">Telemetry Format</h2>
          <div className="bg-muted p-6 rounded-lg">
            <h3 className="text-lg font-medium mb-3">Required Fields</h3>
            <ul className="space-y-2 list-disc pl-5">
              <li><code className="bg-background px-1.5 py-0.5 rounded">robotId</code>: Your robot's ID (UUID format)</li>
              <li><code className="bg-background px-1.5 py-0.5 rounded">api-key</code>: Your API key (in request header)</li>
            </ul>
            
            <h3 className="text-lg font-medium mt-6 mb-3">Optional Fields</h3>
            <ul className="space-y-2 list-disc pl-5">
              <li><code className="bg-background px-1.5 py-0.5 rounded">batteryLevel</code>: Battery percentage (0-100)</li>
              <li><code className="bg-background px-1.5 py-0.5 rounded">temperature</code>: Temperature in Celsius</li>
              <li><code className="bg-background px-1.5 py-0.5 rounded">status</code>: "OK", "WARNING", or "ERROR" (updates robot status)</li>
              <li>
                <code className="bg-background px-1.5 py-0.5 rounded">location</code>: Object with location coordinates
                <ul className="ml-6 mt-2 space-y-1">
                  <li>Preferred format: <code className="bg-background px-1.5 py-0.5 rounded text-xs">{"{ latitude: number, longitude: number }"}</code></li>
                  <li>Also accepted: <code className="bg-background px-1.5 py-0.5 rounded text-xs">{"{ lat: number, lng: number }"}</code></li>
                </ul>
              </li>
              <li><code className="bg-background px-1.5 py-0.5 rounded">timestamp</code>: ISO timestamp (defaults to current time if omitted)</li>
            </ul>
          </div>
        </section>
        
        <section>
          <h2 className="text-2xl font-semibold mb-4">Robot Simulator</h2>
          <div className="bg-muted p-6 rounded-lg">
            <p className="mb-4">
              We provide a simple Node.js client that can simulate a robot sending telemetry data. This is useful for testing your integration.
            </p>
            <ol className="space-y-3 list-decimal pl-5">
              <li>Download the <code className="bg-background px-1.5 py-0.5 rounded">robot-client.js</code> file from your project</li>
              <li>Install dependencies: <code className="bg-background px-1.5 py-0.5 rounded">npm install node-fetch@2</code></li>
              <li>Edit the file to set your robot ID and API key</li>
              <li>Run with: <code className="bg-background px-1.5 py-0.5 rounded">node robot-client.js</code></li>
            </ol>
            <p className="mt-4">
              The simulator will send telemetry data every 10 seconds and simulate battery drain, temperature changes, etc.
            </p>
          </div>
        </section>
        
        {/* Add the ROS Integration section */}
        <ROSIntegration />
      </div>
    </div>
  );
}
