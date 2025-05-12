import { Button } from "@/components/ui/button";
import { TelemetryExample } from "@/components/integration/TelemetryExample";
import { WebSocketExample } from "@/components/integration/WebSocketExample";
import { ArduinoExample } from "@/components/integration/ArduinoExample";
import { ROSExample } from "@/components/integration/ROSExample";
import { MQTTExample } from "@/components/integration/MQTTExample";
import { EmailNotifications } from "@/components/integration/EmailNotifications";
import { CustomTelemetryExample } from "@/components/integration/CustomTelemetryExample";
import { MainLayout } from "@/components/layout/MainLayout";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { useState } from "react";
import { Link } from "react-router-dom";
import { ArrowRight, ChevronDown, ChevronUp, Code, Cpu, Link2, MapPin, MessageSquare, Activity, Mail, Zap } from "lucide-react";

export default function IntegrationGuide() {
  const [activeTab, setActiveTab] = useState("overview");
  const [expandedSections, setExpandedSections] = useState<string[]>([]);

  const toggleSection = (section: string) => {
    setExpandedSections(prev => 
      prev.includes(section) 
        ? prev.filter(s => s !== section) 
        : [...prev, section]
    );
  };

  const isSectionExpanded = (section: string) => {
    return expandedSections.includes(section);
  };

  return (
    <MainLayout>
      <div className="container py-10">
        <div className="mb-10">
          <h1 className="text-3xl font-bold tracking-tight mb-4">Robot Integration Guide</h1>
          <p className="text-muted-foreground max-w-3xl">
            Follow these instructions to connect your robots to the platform. Each robot will need its API key for authentication when sending telemetry data.
          </p>
          <div className="flex items-center gap-2 mt-4 text-sm">
            <Mail className="h-4 w-4 text-primary" />
            <span>Need help with integration? Contact us at </span>
            <a href="mailto:support@robometrics.io" className="text-primary hover:underline">
              support@robometrics.io
            </a>
          </div>
        </div>

        <Tabs value={activeTab} onValueChange={setActiveTab} className="mb-10">
          <TabsList className="mb-6">
            <TabsTrigger value="overview">Overview</TabsTrigger>
            <TabsTrigger value="arduino">Arduino/ESP32</TabsTrigger>
            <TabsTrigger value="ros">ROS</TabsTrigger>
            <TabsTrigger value="mqtt">MQTT</TabsTrigger>
            <TabsTrigger value="http">HTTP API</TabsTrigger>
            <TabsTrigger value="websocket">WebSocket</TabsTrigger>
            <TabsTrigger value="custom">Custom Telemetry</TabsTrigger>
            <TabsTrigger value="notifications">Notifications</TabsTrigger>
          </TabsList>

          <TabsContent value="overview">
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
                      <Link to="/dashboard">Go to Dashboard</Link>
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
                <h2 className="text-2xl font-semibold mb-4">Integration Methods</h2>
                <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-6">
                  <Button
                    variant="outline"
                    className="h-auto flex-col items-start p-4 text-left"
                    onClick={() => setActiveTab("arduino")}
                  >
                    <div className="flex w-full items-center justify-between mb-2">
                      <Cpu className="h-5 w-5" />
                      <ArrowRight className="h-4 w-4 text-muted-foreground" />
                    </div>
                    <div className="space-y-1">
                      <h3 className="font-medium">Arduino/ESP32</h3>
                      <p className="text-sm text-muted-foreground">
                        Connect Arduino or ESP32 microcontrollers
                      </p>
                    </div>
                  </Button>
                  
                  <Button
                    variant="outline"
                    className="h-auto flex-col items-start p-4 text-left"
                    onClick={() => setActiveTab("ros")}
                  >
                    <div className="flex w-full items-center justify-between mb-2">
                      <svg className="h-5 w-5" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                        <circle cx="12" cy="12" r="11" stroke="currentColor" strokeWidth="2" fill="none" />
                        <path d="M5 12H19" stroke="currentColor" strokeWidth="2" />
                        <path d="M12 5V19" stroke="currentColor" strokeWidth="2" />
                        <circle cx="12" cy="12" r="3" fill="currentColor" />
                      </svg>
                      <ArrowRight className="h-4 w-4 text-muted-foreground" />
                    </div>
                    <div className="space-y-1">
                      <h3 className="font-medium">ROS Integration</h3>
                      <p className="text-sm text-muted-foreground">
                        Connect ROS1/ROS2 robots
                      </p>
                    </div>
                  </Button>
                  
                  <Button
                    variant="outline"
                    className="h-auto flex-col items-start p-4 text-left"
                    onClick={() => setActiveTab("mqtt")}
                  >
                    <div className="flex w-full items-center justify-between mb-2">
                      <MessageSquare className="h-5 w-5" />
                      <ArrowRight className="h-4 w-4 text-muted-foreground" />
                    </div>
                    <div className="space-y-1">
                      <h3 className="font-medium">MQTT Protocol</h3>
                      <p className="text-sm text-muted-foreground">
                        Connect via MQTT broker
                      </p>
                    </div>
                  </Button>
                  
                  <Button
                    variant="outline"
                    className="h-auto flex-col items-start p-4 text-left"
                    onClick={() => setActiveTab("http")}
                  >
                    <div className="flex w-full items-center justify-between mb-2">
                      <Code className="h-5 w-5" />
                      <ArrowRight className="h-4 w-4 text-muted-foreground" />
                    </div>
                    <div className="space-y-1">
                      <h3 className="font-medium">HTTP API</h3>
                      <p className="text-sm text-muted-foreground">
                        Direct HTTP API integration
                      </p>
                    </div>
                  </Button>
                  
                  <Button
                    variant="outline"
                    className="h-auto flex-col items-start p-4 text-left bg-primary/5"
                    onClick={() => setActiveTab("websocket")}
                  >
                    <div className="flex w-full items-center justify-between mb-2">
                      <Zap className="h-5 w-5 text-primary" />
                      <ArrowRight className="h-4 w-4 text-muted-foreground" />
                    </div>
                    <div className="space-y-1">
                      <h3 className="font-medium">WebSocket</h3>
                      <p className="text-sm text-muted-foreground">
                        Real-time bidirectional communication
                      </p>
                    </div>
                  </Button>
                  
                  <Button
                    variant="outline"
                    className="h-auto flex-col items-start p-4 text-left"
                    onClick={() => setActiveTab("custom")}
                  >
                    <div className="flex w-full items-center justify-between mb-2">
                      <Activity className="h-5 w-5" />
                      <ArrowRight className="h-4 w-4 text-muted-foreground" />
                    </div>
                    <div className="space-y-1">
                      <h3 className="font-medium">Custom Telemetry</h3>
                      <p className="text-sm text-muted-foreground">
                        Send custom data fields
                      </p>
                    </div>
                  </Button>
                </div>
              </section>

              <section>
                <h2 className="text-2xl font-semibold mb-4">Key Features</h2>
                <div className="space-y-4">
                  <div
                    className="border rounded-lg p-4 cursor-pointer"
                    onClick={() => toggleSection("map")}
                  >
                    <div className="flex items-center justify-between">
                      <div className="flex items-center gap-2">
                        <MapPin className="h-5 w-5 text-primary" />
                        <h3 className="text-lg font-medium">Live Map View</h3>
                      </div>
                      {isSectionExpanded("map") ? <ChevronUp className="h-5 w-5" /> : <ChevronDown className="h-5 w-5" />}
                    </div>
                    
                    {isSectionExpanded("map") && (
                      <div className="mt-4 pl-7">
                        <p className="text-muted-foreground mb-2">
                          The platform provides real-time visualization of all your robots on an interactive map. To enable this feature:
                        </p>
                        <ul className="list-disc pl-5 space-y-1 text-sm">
                          <li>Make sure to include location data in your telemetry payloads</li>
                          <li>Format: <code className="bg-muted px-1">{"location: { latitude: number, longitude: number }"}</code></li>
                          <li>View the map in the <Link to="/map" className="text-primary hover:underline">Map View</Link> section</li>
                        </ul>
                      </div>
                    )}
                  </div>
                  
                  <div
                    className="border rounded-lg p-4 cursor-pointer"
                    onClick={() => toggleSection("notifications")}
                  >
                    <div className="flex items-center justify-between">
                      <div className="flex items-center gap-2">
                        <MessageSquare className="h-5 w-5 text-primary" />
                        <h3 className="text-lg font-medium">Email Notifications</h3>
                      </div>
                      {isSectionExpanded("notifications") ? <ChevronUp className="h-5 w-5" /> : <ChevronDown className="h-5 w-5" />}
                    </div>
                    
                    {isSectionExpanded("notifications") && (
                      <div className="mt-4 pl-7">
                        <p className="text-muted-foreground mb-2">
                          Receive email notifications when your robots require attention:
                        </p>
                        <ul className="list-disc pl-5 space-y-1 text-sm">
                          <li>Low battery alerts</li>
                          <li>Robots going offline</li>
                          <li>Error conditions</li>
                          <li>Configure alerts in the <Link to="/alerts" className="text-primary hover:underline">Alerts</Link> section</li>
                        </ul>
                        <Button
                          variant="link"
                          size="sm"
                          className="mt-2 px-0"
                          onClick={(e) => {
                            e.stopPropagation();
                            setActiveTab("notifications");
                          }}
                        >
                          View notification details
                        </Button>
                      </div>
                    )}
                  </div>

                  <div
                    className="border rounded-lg p-4 cursor-pointer"
                    onClick={() => toggleSection("api")}
                  >
                    <div className="flex items-center justify-between">
                      <div className="flex items-center gap-2">
                        <Link2 className="h-5 w-5 text-primary" />
                        <h3 className="text-lg font-medium">API Access</h3>
                      </div>
                      {isSectionExpanded("api") ? <ChevronUp className="h-5 w-5" /> : <ChevronDown className="h-5 w-5" />}
                    </div>
                    
                    {isSectionExpanded("api") && (
                      <div className="mt-4 pl-7">
                        <p className="text-muted-foreground mb-2">
                          Access robot data and control functionality via our REST API:
                        </p>
                        <ul className="list-disc pl-5 space-y-1 text-sm">
                          <li>Retrieve telemetry history</li>
                          <li>Send commands to robots</li>
                          <li>Manage robot fleet programmatically</li>
                        </ul>
                        <Button
                          variant="link"
                          size="sm"
                          className="mt-2 px-0"
                          onClick={(e) => {
                            e.stopPropagation();
                            setActiveTab("http");
                          }}
                        >
                          View API documentation
                        </Button>
                      </div>
                    )}
                  </div>
                  
                  <div
                    className="border rounded-lg p-4 cursor-pointer bg-primary/5"
                    onClick={() => toggleSection("realtime")}
                  >
                    <div className="flex items-center justify-between">
                      <div className="flex items-center gap-2">
                        <Zap className="h-5 w-5 text-primary" />
                        <h3 className="text-lg font-medium">Real-time Communication</h3>
                      </div>
                      {isSectionExpanded("realtime") ? <ChevronUp className="h-5 w-5" /> : <ChevronDown className="h-5 w-5" />}
                    </div>
                    
                    {isSectionExpanded("realtime") && (
                      <div className="mt-4 pl-7">
                        <p className="text-muted-foreground mb-2">
                          Choose the communication method that best fits your use case:
                        </p>
                        <ul className="list-disc pl-5 space-y-1 text-sm">
                          <li><strong>HTTP API</strong>: Simple request/response for occasional updates</li>
                          <li><strong>WebSockets</strong>: Bidirectional real-time communication for continuous data</li>
                          <li><strong>MQTT</strong>: Lightweight publish/subscribe for IoT devices</li>
                        </ul>
                        <Button
                          variant="link"
                          size="sm"
                          className="mt-2 px-0"
                          onClick={(e) => {
                            e.stopPropagation();
                            setActiveTab("websocket");
                          }}
                        >
                          View WebSocket integration
                        </Button>
                      </div>
                    )}
                  </div>
                </div>
              </section>
            </div>
          </TabsContent>

          <TabsContent value="arduino">
            <ArduinoExample />
          </TabsContent>

          <TabsContent value="ros">
            <ROSExample />
          </TabsContent>

          <TabsContent value="mqtt">
            <MQTTExample />
          </TabsContent>

          <TabsContent value="http">
            <section className="mb-8">
              <h2 className="text-2xl font-semibold mb-4">HTTP API</h2>
              <p className="text-muted-foreground mb-6 max-w-3xl">
                You can send telemetry data using our HTTP API. This is the most direct integration method and works with any programming language or platform that can make HTTP requests.
              </p>
            </section>
            
            <TelemetryExample />
            
            <section className="mt-8">
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
          </TabsContent>
          
          <TabsContent value="websocket">
            <section className="mb-8">
              <h2 className="text-2xl font-semibold mb-4">WebSocket API</h2>
              <p className="text-muted-foreground mb-6 max-w-3xl">
                For real-time communication and continuous data streams, our WebSocket API provides a bidirectional connection with lower latency and reduced overhead compared to HTTP polling.
              </p>
            </section>
            
            <WebSocketExample />
            
            <section className="mt-8">
              <h2 className="text-2xl font-semibold mb-4">WebSocket vs. HTTP</h2>
              <div className="grid md:grid-cols-2 gap-6">
                <div className="bg-muted p-6 rounded-lg">
                  <h3 className="text-lg font-medium mb-3 flex items-center">
                    <Zap className="mr-2 h-5 w-5 text-primary" />
                    WebSocket Benefits
                  </h3>
                  <ul className="space-y-2 list-disc pl-5">
                    <li>Persistent connection for real-time, bidirectional communication</li>
                    <li>Lower latency for time-sensitive applications</li>
                    <li>Reduced overhead for frequent data transmission</li>
                    <li>Better for continuous data streams or high frequency updates</li>
                    <li>More efficient state management for large-scale deployments</li>
                  </ul>
                </div>
                
                <div className="bg-muted p-6 rounded-lg">
                  <h3 className="text-lg font-medium mb-3 flex items-center">
                    <Code className="mr-2 h-5 w-5 text-primary" />
                    When to Use Each
                  </h3>
                  <p className="mb-2">Choose WebSockets for:</p>
                  <ul className="mb-4 list-disc pl-5">
                    <li>Real-time dashboards and monitoring</li>
                    <li>Continuous telemetry from critical systems</li>
                    <li>High-frequency data updates {'>'}1/second</li>
                    <li>Bidirectional communications</li>
                  </ul>
                  
                  <p className="mb-2">Choose HTTP API for:</p>
                  <ul className="list-disc pl-5">
                    <li>Simpler implementation needs</li>
                    <li>Infrequent, periodic updates</li>
                    <li>Systems with limited connectivity</li>
                    <li>One-time data submissions</li>
                  </ul>
                </div>
              </div>
            </section>
          </TabsContent>
          
          <TabsContent value="custom">
            <CustomTelemetryExample />
          </TabsContent>

          <TabsContent value="notifications">
            <EmailNotifications />
          </TabsContent>
        </Tabs>
      </div>
    </MainLayout>
  );
}
