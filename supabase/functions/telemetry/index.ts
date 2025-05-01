
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

    console.log("Headers received:", JSON.stringify([...req.headers.entries()]));
    
    // Check API key in header - support multiple header formats
    const apiKey = req.headers.get("api-key") || 
                  req.headers.get("apikey") || 
                  req.headers.get("API-KEY") || 
                  req.headers.get("APIKEY") ||
                  req.headers.get("Authorization")?.replace("Bearer ", "");
    
    console.log("API Key detected:", apiKey ? `${apiKey.substring(0, 5)}...` : "None");
    
    if (!apiKey) {
      return new Response(
        JSON.stringify({ 
          error: "API Key is required", 
          details: "Please provide your API key in the 'api-key' header",
          receivedHeaders: Object.fromEntries([...req.headers.entries()].map(([k]) => [k, '(hidden)']))
        }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
      );
    }

    console.log("Parsing request body...");
    const telemetryData = await req.json();
    const { robotId, batteryLevel, temperature, status, location, timestamp } = telemetryData;
    
    console.log("Received telemetry for robotId:", robotId);

    if (!robotId) {
      return new Response(
        JSON.stringify({ error: "Robot ID is required" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 400 }
      );
    }

    console.log(`Processing telemetry for robot ${robotId}`);
    console.log(`API key provided: ${apiKey.substring(0, 5)}...`);

    // Find the user who owns the API key
    const { data: profileData, error: profileError } = await supabaseClient
      .from("profiles")
      .select("id")
      .eq("api_key", apiKey)
      .single();

    if (profileError || !profileData) {
      console.error("Profile lookup error:", profileError?.message || "Invalid API key");
      return new Response(
        JSON.stringify({ 
          error: "Invalid API key",
          details: "The provided API key could not be found in any user profile"
        }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
      );
    }

    console.log(`Found user profile with ID: ${profileData.id}`);

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
          console.error(`Robot ${robotId} does not belong to user ${profileData.id}`);
          return new Response(
            JSON.stringify({ 
              error: "Invalid robot ID or you don't have access to this robot", 
              details: "Please use one of your robot IDs"
            }),
            { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
          );
        }
      } else {
        console.error("No robots found in system");
        return new Response(
          JSON.stringify({ error: "No robots found in system" }),
          { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
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
      // We'll use status to update the robot's status below
    };

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

    console.log(`Successfully inserted telemetry data for robot ${robotId}`);

    // Update robot status (last_ping, battery_level, etc.)
    await supabaseClient
      .from("robots")
      .update({
        status: status === "ERROR" ? "offline" : status === "WARNING" ? "warning" : "online",
        battery_level: batteryLevel,
        temperature: temperature,
        location: locationData,
        last_ping: new Date().toISOString()
      })
      .eq("id", robotId);

    return new Response(
      JSON.stringify({ 
        success: true, 
        message: "Telemetry data received",
        robotId: robotId,
        timestamp: new Date().toISOString()
      }),
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
