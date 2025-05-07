
import { serve } from "https://deno.land/std@0.168.0/http/server.ts";
import { createClient } from "https://esm.sh/@supabase/supabase-js@2.43.0";

const corsHeaders = {
  "Access-Control-Allow-Origin": "*",
  "Access-Control-Allow-Headers": "authorization, x-client-info, apikey, api-key, content-type",
};

serve(async (req) => {
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
    const { robotId, batteryLevel, temperature, status, location, timestamp, customTelemetry } = telemetryData;

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

    // Process and validate telemetry data
    // Convert location from client format (lat/lng) to database format (latitude/longitude) if needed
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
      // We'll use status to update the robot's status below
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

    // Check for custom telemetry types and validate against user's allowed types
    if (customTelemetry && typeof customTelemetry === 'object') {
      const userCustomTypes = profileData.custom_telemetry_types || [];
      const incomingTypes = Object.keys(customTelemetry);
      
      // Log the custom telemetry data for debugging
      console.log("Received custom telemetry:", JSON.stringify(customTelemetry));
      console.log("User custom telemetry types:", JSON.stringify(userCustomTypes));
      
      // Create alerts for any custom telemetry that exceeds thresholds
      // We'll need to fetch the custom alerts from the user's profile
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
    const { data, error } = await supabaseClient
      .from("telemetry")
      .insert([telemetry]);

    if (error) {
      console.error("Telemetry insert error:", error);
      return new Response(
        JSON.stringify({ error: "Failed to insert telemetry", details: error }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 500 }
      );
    }

    // Update robot status (last_ping, battery_level, etc.)
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
