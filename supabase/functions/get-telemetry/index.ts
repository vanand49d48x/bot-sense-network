
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
    if (req.method !== "GET") {
      return new Response(
        JSON.stringify({ error: "Method not allowed" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 405 }
      );
    }

    const supabaseClient = createClient(
      Deno.env.get("SUPABASE_URL") ?? "",
      Deno.env.get("SUPABASE_SERVICE_ROLE_KEY") ?? ""
    );

    // Extract the API key from header - check multiple header names
    const apiKey = req.headers.get("apikey") || 
                  req.headers.get("api-key") || 
                  req.headers.get("Authorization")?.replace("Bearer ", "") ||
                  req.headers.get("authorization")?.replace("Bearer ", "");
    
    if (!apiKey) {
      return new Response(
        JSON.stringify({ 
          error: "API Key is required", 
          headers: Object.fromEntries(req.headers),
          message: "Please include your API key in the 'api-key' header"
        }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
      );
    }

    // Find the user who owns this API key
    const { data: profileData, error: profileError } = await supabaseClient
      .from("profiles")
      .select("id")
      .eq("api_key", apiKey)
      .single();

    if (profileError || !profileData) {
      console.error("API key not found:", profileError);
      return new Response(
        JSON.stringify({ 
          error: "Invalid API key", 
          message: "API key not found in user profiles"
        }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
      );
    }

    console.log(`Found user with ID ${profileData.id} for this API key`);

    // Extract the robot ID from the URL path
    const url = new URL(req.url);
    const pathParts = url.pathname.split('/');
    // Expected format: /robots/{robotId}/telemetry
    const robotIdFromPath = pathParts.length >= 4 ? pathParts[3] : null;

    if (!robotIdFromPath) {
      return new Response(
        JSON.stringify({ error: "Robot ID is required in the URL path" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 400 }
      );
    }

    // Verify the robot exists and belongs to the user
    const { data: robotData, error: robotError } = await supabaseClient
      .from("robots")
      .select("id, user_id")
      .eq("id", robotIdFromPath)
      .eq("user_id", profileData.id)
      .single();

    if (robotError || !robotData) {
      return new Response(
        JSON.stringify({ 
          error: "Invalid robot ID or you don't have access to this robot", 
          robotIdFromPath,
          userId: profileData.id
        }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
      );
    }

    // Get the 'last' query parameter, default to 100 if not provided
    const last = parseInt(url.searchParams.get("last") || "100");
    const limit = isNaN(last) || last <= 0 ? 100 : (last > 1000 ? 1000 : last);

    // Get telemetry data
    const { data: telemetry, error } = await supabaseClient
      .from("telemetry")
      .select("*")
      .eq("robot_id", robotData.id)
      .order("created_at", { ascending: false })
      .limit(limit);

    if (error) {
      console.error("Telemetry fetch error:", error);
      return new Response(
        JSON.stringify({ error: "Failed to fetch telemetry data", details: error }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 500 }
      );
    }

    return new Response(
      JSON.stringify({ 
        success: true, 
        data: telemetry.map(t => ({
          id: t.id,
          robotId: t.robot_id,
          batteryLevel: t.battery_level,
          temperature: t.temperature,
          status: t.error_codes && t.error_codes.length > 0 ? "ERROR" : 
                 t.warning_codes && t.warning_codes.length > 0 ? "WARNING" : "OK",
          location: t.location,
          timestamp: t.created_at
        }))
      }),
      { headers: { ...corsHeaders, "Content-Type": "application/json" } }
    );
  } catch (error) {
    console.error("Function error:", error);
    return new Response(
      JSON.stringify({ error: error.message }),
      { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 500 }
    );
  }
});
