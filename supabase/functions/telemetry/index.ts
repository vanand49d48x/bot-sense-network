
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
    const { robotId, batteryLevel, temperature, status, location, timestamp } = telemetryData;

    if (!robotId) {
      return new Response(
        JSON.stringify({ error: "Robot ID is required" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 400 }
      );
    }

    // Verify the robot exists and the API key matches
    // First try to find it using the direct UUID
    let robotQuery = supabaseClient
      .from("robots")
      .select("id, api_key")
      .eq("id", robotId);
      
    let { data: robot, error: robotError } = await robotQuery.single();

    // If the UUID doesn't match directly, try to map it using a more flexible format
    if (!robot) {
      // Let's log the error for diagnosis
      console.log(`Could not find robot with direct ID: ${robotId}`);
      
      // Get all robots and check if any match the ID pattern
      const { data: allRobots, error: listError } = await supabaseClient
        .from("robots")
        .select("id, api_key");
        
      if (!listError && allRobots?.length > 0) {
        // Try to find a robot that matches by name
        robot = allRobots.find(r => r.id === robotId);
        
        if (!robot) {
          return new Response(
            JSON.stringify({ 
              error: "Invalid robot ID", 
              details: "Please use one of the following IDs: " + allRobots.map(r => r.id).join(", ")
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

    // Check if API key matches
    if (robot.api_key !== apiKey) {
      return new Response(
        JSON.stringify({ error: "Invalid API key" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
      );
    }

    // Process and validate telemetry data
    const telemetry = {
      robot_id: robot.id,
      battery_level: batteryLevel || null,
      temperature: temperature || null,
      location: location || null,
      status: status || "OK",
      timestamp: timestamp || new Date().toISOString()
    };

    // Insert telemetry data
    const { data, error } = await supabaseClient
      .from("telemetry")
      .insert([telemetry]);

    if (error) {
      return new Response(
        JSON.stringify({ error: "Failed to insert telemetry", details: error }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 500 }
      );
    }

    // Update robot status (last_ping, battery_level, etc.)
    await supabaseClient
      .from("robots")
      .update({
        status: status === "ERROR" ? "offline" : status === "WARNING" ? "warning" : "online",
        battery_level: batteryLevel,
        temperature: temperature,
        location: location,
        last_ping: new Date().toISOString()
      })
      .eq("id", robot.id);

    return new Response(
      JSON.stringify({ success: true, message: "Telemetry data received" }),
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
