
import { serve } from "https://deno.land/std@0.168.0/http/server.ts";
import { createClient } from "https://esm.sh/@supabase/supabase-js@2.43.0";

const corsHeaders = {
  "Access-Control-Allow-Origin": "*",
  "Access-Control-Allow-Headers": "authorization, x-client-info, apikey, api-key, content-type",
};

serve(async (req) => {
  console.log("🔔 Telemetry function invoked");
  console.log(`Request method: ${req.method}`);
  console.log(`Request URL: ${req.url}`);
  
  // Handle CORS preflight requests
  if (req.method === "OPTIONS") {
    console.log("Handling OPTIONS request (CORS preflight)");
    return new Response(null, { headers: corsHeaders });
  }

  try {
    console.log("Creating Supabase client");
    const supabaseClient = createClient(
      Deno.env.get("SUPABASE_URL") ?? "",
      Deno.env.get("SUPABASE_SERVICE_ROLE_KEY") ?? ""
    );
    console.log("Supabase client created successfully");

    if (req.method !== "POST") {
      console.log(`Invalid method: ${req.method}, expected POST`);
      return new Response(
        JSON.stringify({ error: "Method not allowed" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 405 }
      );
    }

    // Enhanced logging for headers
    console.log("Request headers:", Object.fromEntries(req.headers.entries()));
    
    // Check API key in header with more flexible options
    const apiKey = req.headers.get("api-key") || 
                  req.headers.get("apikey") || 
                  req.headers.get("authorization") ||
                  req.headers.get("Authorization");
                  
    console.log("API Key detected:", apiKey ? "✅ Present" : "❌ Missing");
    
    if (!apiKey) {
      console.log("Authentication failed: No API key provided in headers");
      return new Response(
        JSON.stringify({ 
          error: "API Key is required", 
          details: "Please add one of these headers to your request: 'api-key', 'apikey', or 'authorization'",
          received_headers: Object.keys(Object.fromEntries(req.headers.entries()))
        }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
      );
    }

    // Log request body details
    let telemetryData;
    try {
      telemetryData = await req.json();
      console.log("Request body successfully parsed:", JSON.stringify(telemetryData));
    } catch (parseError) {
      console.error("Failed to parse request body:", parseError);
      return new Response(
        JSON.stringify({ error: "Invalid JSON in request body" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 400 }
      );
    }
    
    const { robotId, batteryLevel, temperature, status, location, timestamp, customTelemetry } = telemetryData;
    
    console.log("Received telemetry for robot:", robotId);
    console.log("Telemetry details:", {
      batteryLevel,
      temperature,
      status,
      location,
      timestamp,
      hasCustomTelemetry: !!customTelemetry
    });

    if (!robotId) {
      console.log("Missing required field: robotId");
      return new Response(
        JSON.stringify({ error: "Robot ID is required" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 400 }
      );
    }

    // Find the user who owns the API key
    console.log("Looking up user profile for API key");
    const { data: profileData, error: profileError } = await supabaseClient
      .from("profiles")
      .select("id, custom_telemetry_types")
      .eq("api_key", apiKey)
      .single();

    if (profileError || !profileData) {
      console.error("API key validation failed:", profileError || "No matching profile found");
      console.log("API Key used:", apiKey.substring(0, 5) + "..." + apiKey.substring(apiKey.length - 5));
      
      // Try to find any profile with an API key for debugging
      console.log("Attempting to debug API key issue...");
      const { data: allProfiles } = await supabaseClient
        .from("profiles")
        .select("id, api_key")
        .not("api_key", "is", null)
        .limit(5);
        
      console.log(`Found ${allProfiles?.length || 0} profiles with API keys`);
      if (allProfiles?.length > 0) {
        const maskedKeys = allProfiles.map(p => ({
          id: p.id,
          key_preview: p.api_key ? 
            `${p.api_key.substring(0, 5)}...${p.api_key.substring(p.api_key.length - 5)}` : 
            'No key'
        }));
        console.log("Sample API keys in database:", maskedKeys);
      }
      
      return new Response(
        JSON.stringify({ 
          error: "Invalid API key", 
          details: "The provided API key was not found in our system",
          debug_info: !profileData ? "No profile found" : "Profile lookup error"
        }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
      );
    }

    console.log("Found profile for API key, user ID:", profileData.id);

    // Verify the robot exists and belongs to the user
    console.log(`Verifying robot ${robotId} belongs to user ${profileData.id}`);
    let { data: robot, error: robotError } = await supabaseClient
      .from("robots")
      .select("id, user_id")
      .eq("id", robotId)
      .eq("user_id", profileData.id)
      .single();

    if (!robot) {
      // Try to find any robot with this ID
      console.log(`Robot ${robotId} not found for user ${profileData.id}, checking all robots...`);
      const { data: allRobots, error: listError } = await supabaseClient
        .from("robots")
        .select("id, user_id");
        
      if (!listError && allRobots?.length > 0) {
        console.log(`Found ${allRobots.length} robots in database`);
        // Try to find a robot that matches
        robot = allRobots.find(r => r.id === robotId);
        
        if (!robot) {
          console.error(`Robot ID ${robotId} not found in any user's robots`);
          return new Response(
            JSON.stringify({ 
              error: "Robot not found", 
              details: "The specified robot ID was not found in the system"
            }),
            { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 404 }
          );
        }
        
        if (robot.user_id !== profileData.id) {
          console.error(`Robot ${robotId} belongs to user ${robot.user_id}, not ${profileData.id}`);
          return new Response(
            JSON.stringify({ 
              error: "Access denied", 
              details: "You don't have access to this robot"
            }),
            { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 403 }
          );
        }
      } else {
        console.error("No robots found in system or database error:", listError);
        return new Response(
          JSON.stringify({ 
            error: "Database error", 
            details: "Could not retrieve robot information"
          }),
          { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 500 }
        );
      }
    }

    console.log(`Verified robot ${robotId} belongs to user ${profileData.id}`);

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
    
    // Insert telemetry data
    console.log("Inserting telemetry data into database");
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
    console.log(`Updating robot ${robotId} status information`);
    const robotUpdate = {
      status: status === "ERROR" ? "offline" : status === "WARNING" ? "warning" : "online",
      battery_level: batteryLevel,
      temperature: temperature,
      location: locationData,
      last_ping: new Date().toISOString(),
      telemetry_data: telemetryDataJson // Store the complete telemetry payload
    };
    
    const { error: updateError } = await supabaseClient
      .from("robots")
      .update(robotUpdate)
      .eq("id", robotId);
      
    if (updateError) {
      console.error("Robot status update error:", updateError);
      // Don't fail the whole request because of this - just log it
    }

    console.log(`Successfully processed telemetry for robot ${robotId}`);

    return new Response(
      JSON.stringify({ success: true, message: "Telemetry data received", alertsProcessed: true }),
      { headers: { ...corsHeaders, "Content-Type": "application/json" } }
    );
  } catch (error) {
    console.error("Fatal error processing telemetry:", error);
    return new Response(
      JSON.stringify({ 
        error: "Server error", 
        message: error.message,
        stack: error.stack
      }),
      { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 500 }
    );
  }
});
