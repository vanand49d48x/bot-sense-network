
import { serve } from "https://deno.land/std@0.168.0/http/server.ts";
import { createClient } from "https://esm.sh/@supabase/supabase-js@2.43.0";

const corsHeaders = {
  "Access-Control-Allow-Origin": "*",
  "Access-Control-Allow-Headers": "authorization, x-client-info, apikey, api-key, content-type",
};

serve(async (req) => {
  // Handle regular HTTP requests for CORS preflight
  if (req.method === "OPTIONS") {
    return new Response(null, { headers: corsHeaders });
  }
  
  // Check if it's a WebSocket upgrade request
  const upgradeHeader = req.headers.get("upgrade") || "";
  if (upgradeHeader.toLowerCase() !== "websocket") {
    return new Response(JSON.stringify({ error: "Expected WebSocket connection" }), { 
      status: 400, 
      headers: { ...corsHeaders, "Content-Type": "application/json" } 
    });
  }

  try {
    // Extract API key from header or query parameter
    const url = new URL(req.url);
    const apiKey = req.headers.get("api-key") || url.searchParams.get("api-key");
    const robotId = url.searchParams.get("robotId");

    if (!apiKey) {
      return new Response(JSON.stringify({ error: "API Key is required" }), { 
        status: 401, 
        headers: { ...corsHeaders, "Content-Type": "application/json" } 
      });
    }

    if (!robotId) {
      return new Response(JSON.stringify({ error: "Robot ID is required" }), { 
        status: 400, 
        headers: { ...corsHeaders, "Content-Type": "application/json" } 
      });
    }

    // Create Supabase client
    const supabaseClient = createClient(
      Deno.env.get("SUPABASE_URL") ?? "",
      Deno.env.get("SUPABASE_SERVICE_ROLE_KEY") ?? ""
    );

    // Validate API key
    const { data: profileData, error: profileError } = await supabaseClient
      .from("profiles")
      .select("id, custom_telemetry_types")
      .eq("api_key", apiKey)
      .single();

    if (profileError || !profileData) {
      return new Response(JSON.stringify({ error: "Invalid API key" }), { 
        status: 401, 
        headers: { ...corsHeaders, "Content-Type": "application/json" } 
      });
    }

    // Verify robot ownership
    const { data: robot, error: robotError } = await supabaseClient
      .from("robots")
      .select("id, user_id")
      .eq("id", robotId)
      .eq("user_id", profileData.id)
      .single();

    if (robotError || !robot) {
      return new Response(
        JSON.stringify({ error: "Invalid robot ID or you don't have access to this robot" }), { 
          status: 401, 
          headers: { ...corsHeaders, "Content-Type": "application/json" } 
        }
      );
    }

    // If we got here, authentication is successful, upgrade to WebSocket
    const { socket, response } = Deno.upgradeWebSocket(req);
    console.log(`WebSocket connection established for robot: ${robotId}`);

    // Process incoming messages
    socket.onopen = () => {
      socket.send(JSON.stringify({ status: "connected", message: "WebSocket connection established" }));
    };

    socket.onmessage = async (event) => {
      try {
        // Parse the incoming telemetry data
        const telemetryData = JSON.parse(event.data);
        console.log(`Received telemetry data for robot ${robotId}:`, telemetryData);

        // Validate and extract the data
        const { batteryLevel, temperature, status, location, customTelemetry } = telemetryData;

        // Process and validate telemetry data similar to the HTTP endpoint
        let locationData = location;
        if (location && (location.lat !== undefined || location.lng !== undefined)) {
          locationData = {
            latitude: location.lat || 0,
            longitude: location.lng || 0
          };
        }

        // Prepare telemetry object
        const telemetry = {
          robot_id: robotId,
          battery_level: batteryLevel || null,
          temperature: temperature || null,
          location: locationData || null,
          motor_status: customTelemetry ? JSON.stringify(customTelemetry) : null,
        };

        // Store raw telemetry data
        let telemetryDataJson = {};
        
        // Add all fields from the original request
        for (const [key, value] of Object.entries(telemetryData)) {
          if (key !== 'robotId') { // Skip robotId as it's redundant
            telemetryDataJson[key] = value;
          }
        }

        // Process custom telemetry and check for alerts
        if (customTelemetry && typeof customTelemetry === 'object') {
          const userCustomTypes = profileData.custom_telemetry_types || [];
          const incomingTypes = Object.keys(customTelemetry);
          
          // Log the custom telemetry data for debugging
          console.log("Received custom telemetry via WebSocket:", JSON.stringify(customTelemetry));
          console.log("User custom telemetry types:", JSON.stringify(userCustomTypes));
          
          // Create alerts for any custom telemetry that exceeds thresholds
          const { data: alertsData } = await supabaseClient
            .from("profiles")
            .select("custom_alerts")
            .eq("id", profileData.id)
            .single();
            
          if (alertsData?.custom_alerts && Array.isArray(alertsData.custom_alerts)) {
            console.log("Custom alert settings:", JSON.stringify(alertsData.custom_alerts));
            
            for (const alert of alertsData.custom_alerts) {
              if (alert.enabled && incomingTypes.includes(alert.type)) {
                const telemetryValue = customTelemetry[alert.type];
                console.log(`Checking alert for ${alert.type}: value=${telemetryValue}, threshold=${alert.threshold}`);
                
                // If this is a numeric value and exceeds threshold, create an alert
                if (typeof telemetryValue === 'number' && telemetryValue >= alert.threshold) {
                  console.log(`Creating alert for ${alert.type} with value ${telemetryValue}`);
                  
                  const { data: alertInsert, error: alertError } = await supabaseClient
                    .from("alerts")
                    .insert({
                      robot_id: robotId,
                      type: alert.type,
                      message: `${alert.type} value of ${telemetryValue} exceeds threshold of ${alert.threshold}`,
                      resolved: false
                    });
                    
                  if (alertError) {
                    console.error("Error creating alert:", alertError);
                    socket.send(JSON.stringify({ error: "Error creating alert", details: alertError }));
                  }
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

        // Insert telemetry data
        const { error } = await supabaseClient
          .from("telemetry")
          .insert([telemetry]);

        if (error) {
          console.error("Telemetry insert error:", error);
          socket.send(JSON.stringify({ error: "Failed to insert telemetry", details: error }));
          return;
        }

        // Update robot status
        const robotUpdate = {
          status: status === "ERROR" ? "offline" : status === "WARNING" ? "warning" : "online",
          battery_level: batteryLevel,
          temperature: temperature,
          location: locationData,
          last_ping: new Date().toISOString(),
          telemetry_data: telemetryDataJson
        };
        
        await supabaseClient
          .from("robots")
          .update(robotUpdate)
          .eq("id", robotId);

        // Send confirmation back to client
        socket.send(JSON.stringify({ 
          success: true, 
          message: "Telemetry data received", 
          timestamp: new Date().toISOString() 
        }));

      } catch (error) {
        console.error("Error processing WebSocket message:", error);
        socket.send(JSON.stringify({ error: "Failed to process telemetry data", details: error.message }));
      }
    };

    socket.onerror = (e) => {
      console.error("WebSocket error:", e);
    };

    socket.onclose = () => {
      console.log(`WebSocket connection closed for robot: ${robotId}`);
    };

    return response;
  } catch (error) {
    console.error("Error setting up WebSocket connection:", error);
    return new Response(JSON.stringify({ error: error.message }), {
      status: 500,
      headers: { ...corsHeaders, "Content-Type": "application/json" },
    });
  }
});
