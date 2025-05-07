
import { useEffect, useState, useRef } from "react";
import { supabase } from "@/integrations/supabase/client";
import { useRobotStore } from "@/store/robotStore";
import { useToast } from "@/hooks/use-toast";
import { Radio } from "lucide-react";
import { Button } from "@/components/ui/button";

// Create a RadioOff component since it doesn't exist in lucide-react
const RadioOff = ({ className = "" }) => (
  <svg
    xmlns="http://www.w3.org/2000/svg"
    width="24"
    height="24"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
    className={className || "lucide lucide-circle"}
  >
    <circle cx="12" cy="12" r="10" />
  </svg>
);

interface TelemetryConnectorProps {
  robotId: string;
  apiKey: string;
}

export function TelemetryConnector({ robotId, apiKey }: TelemetryConnectorProps) {
  const [socket, setSocket] = useState<WebSocket | null>(null);
  const [connected, setConnected] = useState(false);
  const [connecting, setConnecting] = useState(false);
  const [reconnectAttempts, setReconnectAttempts] = useState(0);
  const { updateRobotFromTelemetry } = useRobotStore();
  const { toast } = useToast();
  const reconnectTimerRef = useRef<number | null>(null);
  
  useEffect(() => {
    // Clean up function to be called on unmount
    const cleanup = () => {
      if (socket) {
        socket.close();
      }
      if (reconnectTimerRef.current) {
        clearTimeout(reconnectTimerRef.current);
      }
    };
    
    const connect = () => {
      if (connecting) return;
      
      setConnecting(true);
      
      // Create WebSocket connection to our edge function
      const wsUrl = `wss://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry`;
      const ws = new WebSocket(wsUrl);
      
      ws.onopen = () => {
        console.log("WebSocket connection established");
        setConnecting(false);
        
        // Authenticate with the connection
        ws.send(JSON.stringify({
          type: "authenticate",
          robotId,
          apiKey
        }));
      };
      
      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          console.log("WebSocket message received:", data);
          
          if (data.error) {
            toast({
              title: "WebSocket Error",
              description: data.error,
              variant: "destructive"
            });
            setConnected(false);
          } else if (data.type === "connected") {
            setConnected(true);
            setReconnectAttempts(0);
            
            toast({
              title: "WebSocket Connected",
              description: `Real-time connection established for robot ${data.robotId}`,
              duration: 3000
            });
          } else if (data.type === "telemetry_received") {
            // Telemetry acknowledgement - no need to update UI here
          }
        } catch (e) {
          console.error("Error parsing WebSocket message:", e);
        }
      };
      
      ws.onerror = (error) => {
        console.error("WebSocket error:", error);
        setConnected(false);
        setConnecting(false);
      };
      
      ws.onclose = (event) => {
        console.log("WebSocket connection closed:", event.code, event.reason);
        setConnected(false);
        setConnecting(false);
        
        // Reconnect with exponential backoff
        const maxReconnectAttempts = 5;
        if (reconnectAttempts < maxReconnectAttempts) {
          const delay = Math.min(1000 * (2 ** reconnectAttempts), 30000);
          console.log(`Reconnecting in ${delay}ms, attempt ${reconnectAttempts + 1}/${maxReconnectAttempts}`);
          
          reconnectTimerRef.current = window.setTimeout(() => {
            setReconnectAttempts(prev => prev + 1);
            connect();
          }, delay);
        }
      };
      
      setSocket(ws);
    };
    
    if (robotId && apiKey) {
      connect();
    }
    
    // Return cleanup function
    return cleanup;
  }, [robotId, apiKey, reconnectAttempts, toast, connecting, updateRobotFromTelemetry]);
  
  const sendTestTelemetry = () => {
    if (!socket || socket.readyState !== WebSocket.OPEN) {
      toast({
        title: "WebSocket not connected",
        description: "Cannot send test telemetry",
        variant: "destructive"
      });
      return;
    }
    
    const testData = {
      batteryLevel: 75 + Math.floor(Math.random() * 20),
      temperature: 25 + Math.floor(Math.random() * 10),
      location: {
        latitude: 37.7749 + (Math.random() * 0.01),
        longitude: -122.4194 + (Math.random() * 0.01)
      },
      customTelemetry: {
        motorSpeed: 1000 + Math.floor(Math.random() * 500),
        signalStrength: 80 + Math.floor(Math.random() * 20),
        ABCD: 30 + Math.floor(Math.random() * 20)
      }
    };
    
    socket.send(JSON.stringify({
      type: "telemetry",
      payload: testData
    }));
    
    // Also update local state immediately for responsiveness
    // This ensures the UI updates even before the server confirms
    updateRobotFromTelemetry(robotId, testData);
    
    toast({
      title: "Test telemetry sent",
      description: "Check the robot status to see the updates"
    });
  };
  
  return (
    <div className="flex flex-col gap-2">
      <div className="flex items-center gap-2">
        <div className="flex items-center">
          {connected ? (
            <Radio className="h-4 w-4 text-green-500 animate-pulse" />
          ) : (
            <RadioOff className="h-4 w-4 text-gray-400" />
          )}
          <span className="ml-2 text-sm">
            {connected ? "Connected" : connecting ? "Connecting..." : "Disconnected"}
          </span>
        </div>
        
        {connected && (
          <Button 
            variant="outline" 
            size="sm" 
            onClick={sendTestTelemetry}
            className="ml-auto"
          >
            Send Test Data
          </Button>
        )}
      </div>
      
      {reconnectAttempts > 0 && !connected && (
        <p className="text-xs text-muted-foreground">
          Reconnection attempt {reconnectAttempts}/5
        </p>
      )}
    </div>
  );
}
