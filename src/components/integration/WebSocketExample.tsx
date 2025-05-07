
import React from "react";
import { CodeBlock } from "@/components/ui/code";

export function WebSocketExample() {
  const webSocketCode = `
// Initialize WebSocket connection
const connectRobotWebSocket = (robotId, apiKey) => {
  // WebSocket URL for your Supabase Edge Function
  const wsUrl = "wss://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry";
  
  // Create WebSocket connection
  const socket = new WebSocket(wsUrl);
  
  socket.onopen = () => {
    console.log("WebSocket connection established");
    
    // Authenticate the connection
    socket.send(JSON.stringify({
      type: "authenticate",
      robotId: robotId,
      apiKey: apiKey
    }));
  };
  
  socket.onmessage = (event) => {
    const data = JSON.parse(event.data);
    console.log("Received from server:", data);
    
    if (data.error) {
      console.error("WebSocket error:", data.error);
    } else if (data.type === "connected") {
      console.log("Authentication successful!");
      
      // Start sending telemetry data periodically
      setInterval(() => {
        sendTelemetry(socket, {
          batteryLevel: 85,
          temperature: 32.5,
          status: "OK",
          location: { latitude: 37.7749, longitude: -122.4194 },
          customTelemetry: {
            motorSpeed: 1200,
            ABCD: 28,
            proximityDistance: 35
          }
        });
      }, 5000);
    }
  };
  
  socket.onerror = (error) => {
    console.error("WebSocket error:", error);
  };
  
  socket.onclose = (event) => {
    console.log("WebSocket connection closed:", event.code, event.reason);
  };
  
  return socket;
};

// Send telemetry data through WebSocket
const sendTelemetry = (socket, telemetryData) => {
  if (socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify({
      type: "telemetry",
      payload: telemetryData
    }));
  } else {
    console.error("WebSocket is not open");
  }
};

// Usage example:
// const socket = connectRobotWebSocket("your-robot-id", "your-api-key");
// 
// // To disconnect:
// // socket.close();
`;

  return (
    <section className="space-y-4">
      <h2 className="text-2xl font-semibold mb-4">WebSocket Integration</h2>
      
      <div className="text-muted-foreground mb-4">
        <p className="mb-2">
          For real-time telemetry transmission, we recommend using WebSockets. 
          This allows for persistent connections and bidirectional communication between your robots and our platform.
        </p>
        <p>
          WebSockets reduce overhead compared to HTTP requests and are ideal for:
        </p>
        <ul className="list-disc pl-6 mt-2 space-y-1">
          <li>Continuous telemetry streaming</li>
          <li>Low-latency applications</li>
          <li>Bandwidth-efficient data transmission</li>
          <li>Reducing connection setup/teardown overhead</li>
        </ul>
      </div>
      
      <div className="bg-muted rounded-lg p-4">
        <h3 className="font-medium mb-2">WebSocket Connection Example</h3>
        <CodeBlock language="javascript" code={webSocketCode} />
      </div>
      
      <div className="mt-6 bg-muted/50 p-4 rounded-lg border border-dashed">
        <h3 className="font-medium mb-2">WebSocket vs. HTTP API</h3>
        <div className="grid md:grid-cols-2 gap-4">
          <div>
            <h4 className="text-sm font-medium">When to use WebSockets:</h4>
            <ul className="list-disc pl-6 mt-1 text-sm">
              <li>Continuous data streaming</li>
              <li>Low-latency requirements</li>
              <li>Bidirectional communication</li>
              <li>High-frequency telemetry updates</li>
            </ul>
          </div>
          <div>
            <h4 className="text-sm font-medium">When to use HTTP API:</h4>
            <ul className="list-disc pl-6 mt-1 text-sm">
              <li>Occasional updates</li>
              <li>Simple integration needs</li>
              <li>Limited bandwidth environments</li>
              <li>When WebSockets are blocked by network policies</li>
            </ul>
          </div>
        </div>
      </div>
    </section>
  );
}
