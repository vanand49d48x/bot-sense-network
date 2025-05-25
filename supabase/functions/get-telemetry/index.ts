
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

    // Check API key in header
    const apiKey = req.headers.get("api-key");
    if (!apiKey) {
      return new Response(
        JSON.stringify({ error: "API Key is required" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
      );
    }

    // Find the user who owns the API key
    const { data: profileData, error: profileError } = await supabaseClient
      .from("profiles")
      .select("id, telemetry_retention_days")
      .eq("api_key", apiKey)
      .single();

    if (profileError || !profileData) {
      return new Response(
        JSON.stringify({ error: "Invalid API key" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
      );
    }

    // Extract the robot ID from the URL path
    const url = new URL(req.url);
    const pathParts = url.pathname.split('/');
    // Expected format: /v1/robots/{robotId}/telemetry
    const robotId = pathParts.length >= 4 ? pathParts[3] : null;

    if (!robotId) {
      return new Response(
        JSON.stringify({ error: "Robot ID is required in the URL path" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 400 }
      );
    }

    // Verify the robot exists and belongs to the user
    const { data: robot, error: robotError } = await supabaseClient
      .from("robots")
      .select("id")
      .eq("id", robotId)
      .eq("user_id", profileData.id)
      .single();

    if (robotError || !robot) {
      return new Response(
        JSON.stringify({ error: "Invalid robot ID or you don't have access to this robot" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
      );
    }

    // Get query parameters
    const limit = parseInt(url.searchParams.get("limit") || "100");
    const page = parseInt(url.searchParams.get("page") || "1");
    const retentionDays = profileData.telemetry_retention_days || 7;
    
    // Calculate pagination ranges
    const from = (page - 1) * limit;
    const to = page * limit - 1;
    
    // Calculate the date cutoff based on retention days
    const cutoffDate = new Date();
    cutoffDate.setDate(cutoffDate.getDate() - retentionDays);

    // Get telemetry data with pagination and retention period
    const { data: telemetry, error, count } = await supabaseClient
      .from("telemetry")
      .select("*", { count: "exact" })
      .eq("robot_id", robotId)
      .gte("created_at", cutoffDate.toISOString())
      .order("created_at", { ascending: false })
      .range(from, to);

    if (error) {
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
          status: t.error_codes && t.error_codes.length > 0 ? "ERROR" : "OK",
          location: t.location,
          customTelemetry: t.motor_status,
          errorCodes: t.error_codes,
          timestamp: t.created_at
        })),
        pagination: {
          page,
          limit,
          total: count,
          pages: Math.ceil((count || 0) / limit)
        },
        retention: {
          days: retentionDays,
          cutoff: cutoffDate.toISOString()
        }
      }),
      { headers: { ...corsHeaders, "Content-Type": "application/json" } }
    );
  } catch (error) {
    return new Response(
      JSON.stringify({ error: error.message }),
      { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 500 }
    );
  }
});
