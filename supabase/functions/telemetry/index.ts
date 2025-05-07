
import { serve } from "https://deno.land/std@0.168.0/http/server.ts";
import { createClient } from "https://esm.sh/@supabase/supabase-js@2.43.0";

const corsHeaders = {
  "Access-Control-Allow-Origin": "*",
  "Access-Control-Allow-Headers": "authorization, x-client-info, apikey, api-key, content-type",
};

// Track active WebSocket connections
const activeConnections = new Map();

// Handle WebSocket connections
const handleWebSocket = (req) => {
  try {
    const { socket, response } = Deno.upgradeWebSocket(req);
    
    // Set up robot ID/connection mapping
    let robotId = null;
    
    socket.onopen = () => {
      console.log("WebSocket connection established");
    };
    
    socket.onmessage = async (event) => {
      try {
        const data = JSON.parse(event.data);
        
        // If this is an authentication message, store the robot ID
        if (data.type === "authenticate") {
          const { robotId: id, apiKey } = data;
          
          // Verify API key and robot ID
          const supabaseClient = createClient(
            Deno.env.get("SUPABASE_URL") ?? "",
            Deno.env.get("SUPABASE_SERVICE_ROLE_KEY") ?? ""
          );
          
          // Find the user who owns the API key
          const { data: profileData } = await supabaseClient
            .from("profiles")
            .select("id")
            .eq("api_key", apiKey)
            .single();
            
          if (!profileData) {
            socket.send(JSON.stringify({ error: "Invalid API key" }));
            socket.close();
            return;
          }
          
          // Verify the robot exists and belongs to the user
          const { data: robot } = await supabaseClient
            .from("robots")
            .select("id")
            .eq("id", id)
            .eq("user_id", profileData.id)
            .single();
            
          if (!robot) {
            socket.send(JSON.stringify({ error: "Invalid robot ID" }));
            socket.close();
            return;
          }
          
          // Store connection with robot ID
          robotId = id;
          activeConnections.set(robotId, socket);
          console.log(`Registered WebSocket for robot ${robotId}`);
          
          // Acknowledge successful connection
          socket.send(JSON.stringify({ type: "connected", robotId }));
        } 
        // Handle telemetry updates
        else if (data.type === "telemetry" && robotId) {
          // Process telemetry as usual but don't need to respond
          await processTelemetry(data.payload, robotId);
          socket.send(JSON.stringify({ type: "telemetry_received" }));
        }
      } catch (error) {
        console.error("Error processing WebSocket message:", error);
        socket.send(JSON.stringify({ error: "Invalid message format" }));
      }
    };
    
    socket.onclose = () => {
      console.log(`WebSocket connection closed for robot ${robotId}`);
      if (robotId) {
        activeConnections.delete(robotId);
      }
    };
    
    socket.onerror = (error) => {
      console.error("WebSocket error:", error);
    };
    
    return response;
  } catch (error) {
    console.error("Error handling WebSocket connection:", error);
    return new Response(JSON.stringify({ error: "WebSocket connection failed" }), {
      status: 500,
      headers: { ...corsHeaders, "Content-Type": "application/json" }
    });
  }
};

// Process telemetry data
async function processTelemetry(telemetryData, robotId) {
  const supabaseClient = createClient(
    Deno.env.get("SUPABASE_URL") ?? "",
    Deno.env.get("SUPABASE_SERVICE_ROLE_KEY") ?? ""
  );
  
  const { batteryLevel, temperature, status, location, timestamp, customTelemetry } = telemetryData;

  // Process and validate telemetry data
  let locationData = location;
  if (location && (location.lat !== undefined || location.lng !== undefined)) {
    locationData = {
      latitude: location.lat || 0,
      longitude: location.lng || 0
    };
  }

  // Prepare telemetry object with only the fields that exist in the database
  const telemetry = {
    robot_id: robotId,
    battery_level: batteryLevel || null,
    temperature: temperature || null,
    location: locationData || null,
    // Store custom telemetry data if provided
    motor_status: customTelemetry ? JSON.stringify(customTelemetry) : null,
  };

  // Store raw telemetry data
  let telemetryDataJson = {};
  
  // Add all fields from the original request
  // This preserves the complete payload in its original format
  for (const [key, value] of Object.entries(telemetryData)) {
    if (key !== 'robotId') { // Skip robotId as it's redundant
      telemetryDataJson[key] = value;
    }
  }

  // Check for custom telemetry types and handle alerts
  if (customTelemetry && typeof customTelemetry === 'object') {
    // Fetch the user's profile for the robot
    const { data: robot } = await supabaseClient
      .from("robots")
      .select("user_id")
      .eq("id", robotId)
      .single();
      
    if (robot) {
      // Get user's custom alert settings
      const { data: alertsData } = await supabaseClient
        .from("profiles")
        .select("custom_alerts")
        .eq("id", robot.user_id)
        .single();
        
      if (alertsData?.custom_alerts && Array.isArray(alertsData.custom_alerts)) {
        console.log("Custom alert settings:", JSON.stringify(alertsData.custom_alerts));
        const incomingTypes = Object.keys(customTelemetry);
        
        for (const alert of alertsData.custom_alerts) {
          if (alert.enabled && incomingTypes.includes(alert.type)) {
            const telemetryValue = customTelemetry[alert.type];
            console.log(`Checking alert for ${alert.type}: value=${telemetryValue}, threshold=${alert.threshold}`);
            
            // If this is a numeric value and exceeds threshold, create an alert
            if (typeof telemetryValue === 'number' && telemetryValue >= alert.threshold) {
              console.log(`Creating alert for ${alert.type} with value ${telemetryValue}`);
              
              await supabaseClient
                .from("alerts")
                .insert({
                  robot_id: robotId,
                  type: alert.type,
                  message: `${alert.type} value of ${telemetryValue} exceeds threshold of ${alert.threshold}`,
                  resolved: false
                });
            }
          }
        }
      }
      
      // Also check for standard thresholds for ABCD if not in custom alerts
      if (customTelemetry.ABCD !== undefined && typeof customTelemetry.ABCD === 'number') {
        const abcdValue = customTelemetry.ABCD;
        // Default threshold for ABCD if not set in custom alerts
        if (abcdValue > 40) {
          const hasCustomAlert = alertsData?.custom_alerts?.some(alert => 
            alert.type === 'ABCD' && alert.enabled
          );
          
          // Only create a default alert if there's no custom alert for this type
          if (!hasCustomAlert) {
            console.log(`Creating default alert for ABCD with value ${abcdValue}`);
            await supabaseClient.from("alerts").insert({
              robot_id: robotId,
              type: 'ABCD',
              message: `ABCD value of ${abcdValue} exceeds default threshold of 40`,
              resolved: false
            });
          }
        }
      }
    }
  }

  // Insert telemetry data
  await supabaseClient.from("telemetry").insert([telemetry]);

  // Update robot status
  const robotUpdate = {
    status: status === "ERROR" ? "offline" : status === "WARNING" ? "warning" : "online",
    battery_level: batteryLevel,
    temperature: temperature,
    location: locationData,
    last_ping: new Date().toISOString(),
    telemetry_data: telemetryDataJson // Store the complete telemetry payload
  };
  
  await supabaseClient
    .from("robots")
    .update(robotUpdate)
    .eq("id", robotId);
    
  return { success: true };
}

serve(async (req) => {
  // Check for WebSocket connection
  if (req.headers.get("upgrade")?.toLowerCase() === "websocket") {
    return handleWebSocket(req);
  }
  
  // Handle CORS preflight requests
  if (req.method === "OPTIONS") {
    return new Response(null, { headers: corsHeaders });
  }

  try {
    const supabaseClient = createClient(
      Deno.env.get("SUPABASE_URL") ?? "",
      Deno.env.get("SUPABASE_SERVICE_ROLE_KEY") ?? ""
    );

    if (req.method !== "POST") {
      return new Response(
        JSON.stringify({ error: "Method not allowed" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 405 }
      );
    }

    // Check API key in header
    const apiKey = req.headers.get("api-key");
    if (!apiKey) {
      return new Response(
        JSON.stringify({ error: "API Key is required" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
      );
    }

    const telemetryData = await req.json();
    const { robotId } = telemetryData;

    if (!robotId) {
      return new Response(
        JSON.stringify({ error: "Robot ID is required" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 400 }
      );
    }

    // Find the user who owns the API key
    const { data: profileData, error: profileError } = await supabaseClient
      .from("profiles")
      .select("id, custom_telemetry_types")
      .eq("api_key", apiKey)
      .single();

    if (profileError || !profileData) {
      return new Response(
        JSON.stringify({ error: "Invalid API key" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
      );
    }

    // Verify the robot exists and belongs to the user
    let { data: robot, error: robotError } = await supabaseClient
      .from("robots")
      .select("id, user_id")
      .eq("id", robotId)
      .eq("user_id", profileData.id)
      .single();

    if (!robot) {
      // Try to find any robot with this ID
      const { data: allRobots, error: listError } = await supabaseClient
        .from("robots")
        .select("id, user_id");
        
      if (!listError && allRobots?.length > 0) {
        // Try to find a robot that matches
        robot = allRobots.find(r => r.id === robotId);
        
        if (!robot || robot.user_id !== profileData.id) {
          return new Response(
            JSON.stringify({ 
              error: "Invalid robot ID or you don't have access to this robot", 
              details: "Please use one of your robot IDs"
            }),
            { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
          );
        }
      } else {
        return new Response(
          JSON.stringify({ error: "No robots found in system" }),
          { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
        );
      }
    }

    // Process the telemetry data and get result
    const result = await processTelemetry(telemetryData, robotId);

    return new Response(
      JSON.stringify({ success: true, message: "Telemetry data received", alertsProcessed: true }),
      { headers: { ...corsHeaders, "Content-Type": "application/json" } }
    );
  } catch (error) {
    console.error("Error processing telemetry:", error);
    return new Response(
      JSON.stringify({ error: error.message }),
      { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 500 }
    );
  }
});
