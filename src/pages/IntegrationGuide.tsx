
import { Button } from "@/components/ui/button";
import { TelemetryExample } from "@/components/integration/TelemetryExample";

export default function IntegrationGuide() {
  return (
    <div className="container py-10">
      <div className="mb-10">
        <h1 className="text-3xl font-bold tracking-tight mb-4">Robot Integration Guide</h1>
        <p className="text-muted-foreground max-w-3xl">
          Follow these instructions to connect your robots to the platform. Each robot will need its API key for authentication when sending telemetry data.
        </p>
      </div>

      <div className="space-y-8">
        <section>
          <h2 className="text-2xl font-semibold mb-4">Getting Started</h2>
          <div className="grid gap-6 lg:grid-cols-2">
            <div className="space-y-4">
              <h3 className="text-xl font-medium">1. Add your robot</h3>
              <p className="text-muted-foreground">
                First, add your robot to the dashboard. This will generate a unique identifier and API key for your robot.
              </p>
              <Button asChild>
                <a href="/dashboard">Go to Dashboard</a>
              </Button>
            </div>
            <div className="space-y-4">
              <h3 className="text-xl font-medium">2. Find your API key</h3>
              <p className="text-muted-foreground">
                Each robot has a unique API key that must be included in all telemetry requests. 
                Find this in the robot's card by clicking "API Integration".
              </p>
            </div>
          </div>
        </section>

        <section>
          <h2 className="text-2xl font-semibold mb-4">Sending Telemetry</h2>
          <p className="text-muted-foreground mb-6">
            You can send telemetry data using the following endpoint: <code className="bg-muted px-1.5 py-0.5 rounded text-sm">https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry</code>
          </p>
          
          <TelemetryExample />
        </section>

        <section>
          <h2 className="text-2xl font-semibold mb-4">Telemetry Format</h2>
          <div className="bg-muted p-6 rounded-lg">
            <h3 className="text-lg font-medium mb-3">Required Fields</h3>
            <ul className="space-y-2 list-disc pl-5">
              <li><code className="bg-background px-1.5 py-0.5 rounded">robotId</code>: Your robot's ID (UUID format)</li>
              <li><code className="bg-background px-1.5 py-0.5 rounded">api-key</code>: Your robot's API key (in request header)</li>
            </ul>
            
            <h3 className="text-lg font-medium mt-6 mb-3">Optional Fields</h3>
            <ul className="space-y-2 list-disc pl-5">
              <li><code className="bg-background px-1.5 py-0.5 rounded">batteryLevel</code>: Battery percentage (0-100)</li>
              <li><code className="bg-background px-1.5 py-0.5 rounded">temperature</code>: Temperature in Celsius</li>
              <li><code className="bg-background px-1.5 py-0.5 rounded">status</code>: "OK", "WARNING", or "ERROR"</li>
              <li><code className="bg-background px-1.5 py-0.5 rounded">location</code>: Object with latitude and longitude</li>
              <li><code className="bg-background px-1.5 py-0.5 rounded">timestamp</code>: ISO timestamp (defaults to current time if omitted)</li>
            </ul>
          </div>
        </section>
      </div>
    </div>
  );
}
