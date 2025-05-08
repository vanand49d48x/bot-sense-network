
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
    
    // Extract client info for logging
    const clientInfo = req.headers.get("x-client-info") || "unknown";
    const userAgent = req.headers.get("user-agent") || "unknown";
    
    // Create a client identifier for logging
    const clientId = `${clientInfo} | ${userAgent.substring(0, 50)}`;
    
    console.log(`WebSocket connection request from client: ${clientId}`);
    console.log(`Request URL: ${req.url}`);

    // Enhanced validation with more detailed error messages
    if (!apiKey) {
      console.error(`Missing API key in WebSocket connection attempt from client: ${clientId}`);
      return new Response(JSON.stringify({ 
        error: "Authentication error", 
        details: "API Key is required. Please provide it in the 'api-key' header or as a query parameter."
      }), { 
        status: 401, 
        headers: { ...corsHeaders, "Content-Type": "application/json" } 
      });
    }

    if (!robotId) {
      console.error(`Missing robot ID in WebSocket connection attempt from client: ${clientId}`);
      return new Response(JSON.stringify({ 
        error: "Missing parameter", 
        details: "Robot ID is required. Please provide it as the 'robotId' query parameter."
      }), { 
        status: 400, 
        headers: { ...corsHeaders, "Content-Type": "application/json" } 
      });
    }

    // Create Supabase client
    const supabaseClient = createClient(
      Deno.env.get("SUPABASE_URL") ?? "",
      Deno.env.get("SUPABASE_SERVICE_ROLE_KEY") ?? ""
    );

    // Validate API key with enhanced error reporting
    console.log(`Validating API key for WebSocket connection: ${apiKey.substring(0, 8)}... | Client: ${clientId}`);
    const { data: profileData, error: profileError } = await supabaseClient
      .from("profiles")
      .select("id, custom_telemetry_types")
      .eq("api_key", apiKey)
      .single();

    if (profileError || !profileData) {
      console.error(`Invalid API key in WebSocket connection from client ${clientId}: ${profileError?.message || "No matching profile found"}`);
      return new Response(JSON.stringify({ 
        error: "Authentication failed", 
        details: "The provided API key is invalid or has been revoked. Please check your credentials."
      }), { 
        status: 401, 
        headers: { ...corsHeaders, "Content-Type": "application/json" } 
      });
    }

    console.log(`API key validated for user ID: ${profileData.id} | Client: ${clientId}`);

    // Verify robot ownership with enhanced error reporting
    console.log(`Verifying robot ownership: Robot ID ${robotId}, User ID ${profileData.id} | Client: ${clientId}`);
    const { data: robot, error: robotError } = await supabaseClient
      .from("robots")
      .select("id, user_id")
      .eq("id", robotId)
      .eq("user_id", profileData.id)
      .single();

    if (robotError) {
      console.error(`Error verifying robot ownership for client ${clientId}: ${robotError.message}`);
      return new Response(
        JSON.stringify({ 
          error: "Database error", 
          details: "Failed to verify robot ownership due to a database error. Please try again later."
        }), { 
          status: 500, 
          headers: { ...corsHeaders, "Content-Type": "application/json" } 
        }
      );
    }

    if (!robot) {
      console.error(`Robot ownership verification failed for client ${clientId}: Robot ID ${robotId} does not belong to user ${profileData.id}`);
      return new Response(
        JSON.stringify({ 
          error: "Access denied", 
          details: "Invalid robot ID or you don't have access to this robot. Please check the robot ID and ensure it belongs to your account."
        }), { 
          status: 403,
          headers: { ...corsHeaders, "Content-Type": "application/json" } 
        }
      );
    }

    console.log(`Robot ownership verified for client ${clientId}: Robot ID ${robotId} belongs to user ${profileData.id}`);

    // If we got here, authentication is successful, upgrade to WebSocket
    const { socket, response } = Deno.upgradeWebSocket(req);
    console.log(`WebSocket connection established for client ${clientId}, robot: ${robotId}`);

    // Process incoming messages
    socket.onopen = () => {
      socket.send(JSON.stringify({ 
        status: "connected", 
        message: "WebSocket connection established",
        robotId: robotId
      }));
    };

    socket.onmessage = async (event) => {
      try {
        // Parse the incoming telemetry data
        const telemetryData = JSON.parse(event.data);
        console.log(`Received telemetry data for robot ${robotId} from client ${clientId}:`, telemetryData);

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
          console.log(`Received custom telemetry via WebSocket from client ${clientId}:`, JSON.stringify(customTelemetry));
          console.log(`User custom telemetry types for client ${clientId}:`, JSON.stringify(userCustomTypes));
          
          // Create alerts for any custom telemetry that exceeds thresholds
          const { data: alertsData } = await supabaseClient
            .from("profiles")
            .select("custom_alerts")
            .eq("id", profileData.id)
            .single();
            
          if (alertsData?.custom_alerts && Array.isArray(alertsData.custom_alerts)) {
            console.log(`Custom alert settings for client ${clientId}:`, JSON.stringify(alertsData.custom_alerts));
            
            for (const alert of alertsData.custom_alerts) {
              if (alert.enabled && incomingTypes.includes(alert.type)) {
                const telemetryValue = customTelemetry[alert.type];
                console.log(`Checking alert for ${alert.type}: value=${telemetryValue}, threshold=${alert.threshold} | Client: ${clientId}`);
                
                // If this is a numeric value and exceeds threshold, create an alert
                if (typeof telemetryValue === 'number' && telemetryValue >= alert.threshold) {
                  console.log(`Creating alert for ${alert.type} with value ${telemetryValue} | Client: ${clientId}`);
                  
                  const { data: alertInsert, error: alertError } = await supabaseClient
                    .from("alerts")
                    .insert({
                      robot_id: robotId,
                      type: alert.type,
                      message: `${alert.type} value of ${telemetryValue} exceeds threshold of ${alert.threshold}`,
                      resolved: false
                    });
                    
                  if (alertError) {
                    console.error(`Error creating alert for client ${clientId}:`, alertError);
                    socket.send(JSON.stringify({ 
                      warning: "Alert creation failed", 
                      details: "Failed to create alert for threshold violation",
                      error: alertError.message
                    }));
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
                console.log(`Creating default alert for ABCD with value ${abcdValue} | Client: ${clientId}`);
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
          console.error(`Telemetry insert error for client ${clientId}:`, error);
          socket.send(JSON.stringify({ 
            error: "Database error", 
            details: "Failed to insert telemetry data",
            message: error.message 
          }));
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
        
        const { error: updateError } = await supabaseClient
          .from("robots")
          .update(robotUpdate)
          .eq("id", robotId);
          
        if (updateError) {
          console.error(`Robot status update error for client ${clientId}:`, updateError);
          socket.send(JSON.stringify({ 
            warning: "Status update issue", 
            details: "Robot telemetry saved but status update failed",
            error: updateError.message
          }));
        }

        // Send confirmation back to client
        socket.send(JSON.stringify({ 
          success: true, 
          message: "Telemetry data received and processed successfully", 
          timestamp: new Date().toISOString() 
        }));

      } catch (error) {
        console.error(`Error processing WebSocket message from client ${clientId}:`, error);
        socket.send(JSON.stringify({ 
          error: "Processing error", 
          details: "Failed to process telemetry data due to an error in the server",
          message: error.message,
          stack: error.stack
        }));
      }
    };

    socket.onerror = (e) => {
      console.error(`WebSocket error for client ${clientId}:`, e);
      // The socket.onerror handler cannot send messages as the connection may be broken
    };

    socket.onclose = () => {
      console.log(`WebSocket connection closed for client ${clientId}, robot: ${robotId}`);
    };

    return response;
  } catch (error) {
    const clientInfo = req.headers.get("x-client-info") || "unknown";
    const userAgent = req.headers.get("user-agent") || "unknown";
    const clientId = `${clientInfo} | ${userAgent.substring(0, 50)}`;
    console.error(`Error setting up WebSocket connection for client ${clientId}:`, error);
    return new Response(JSON.stringify({ 
      error: "Server error", 
      details: "Failed to establish WebSocket connection due to a server error",
      message: error.message,
      stack: error.stack
    }), {
      status: 500,
      headers: { ...corsHeaders, "Content-Type": "application/json" },
    });
  }
});
