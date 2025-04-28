
import { serve } from "https://deno.land/std@0.168.0/http/server.ts";
import { createClient } from "https://esm.sh/@supabase/supabase-js@2.43.0";

const corsHeaders = {
  "Access-Control-Allow-Origin": "*",
  "Access-Control-Allow-Headers": "authorization, x-client-info, apikey, content-type",
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

    const { robotId, apiKey, ...telemetryData } = await req.json();

    if (!robotId || !apiKey) {
      return new Response(
        JSON.stringify({ error: "Robot ID and API Key are required" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 400 }
      );
    }

    // Verify the robot exists and the API key matches
    const { data: robot, error: robotError } = await supabaseClient
      .from("robots")
      .select("id")
      .eq("id", robotId)
      .single();

    if (robotError || !robot) {
      return new Response(
        JSON.stringify({ error: "Invalid robot ID or API key" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
      );
    }

    // Process and validate telemetry data
    const telemetry = {
      robot_id: robotId,
      battery_level: telemetryData.battery_level || null,
      temperature: telemetryData.temperature || null,
      location: telemetryData.location || null,
      motor_status: telemetryData.motor_status || null,
      error_codes: telemetryData.error_codes || []
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

    return new Response(
      JSON.stringify({ success: true, message: "Telemetry data received" }),
      { headers: { ...corsHeaders, "Content-Type": "application/json" } }
    );
  } catch (error) {
    return new Response(
      JSON.stringify({ error: error.message }),
      { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 500 }
    );
  }
});
