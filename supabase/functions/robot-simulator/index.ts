
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

    const { robotId } = await req.json();

    if (!robotId) {
      return new Response(
        JSON.stringify({ error: "Robot ID is required" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 400 }
      );
    }

    // Get the robot information first
    const { data: robot, error: robotError } = await supabaseClient
      .from("robots")
      .select("*")
      .eq("id", robotId)
      .single();

    if (robotError || !robot) {
      return new Response(
        JSON.stringify({ error: "Robot not found", details: robotError }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 404 }
      );
    }

    // Generate randomized telemetry data
    const batteryDrain = Math.random() * 5; // 0-5% battery drain
    const tempChange = (Math.random() * 4) - 2; // -2 to +2 degrees
    
    const oldLocation = robot.location as { latitude: number, longitude: number };
    const newLocation = {
      latitude: oldLocation.latitude + ((Math.random() - 0.5) * 0.01),
      longitude: oldLocation.longitude + ((Math.random() - 0.5) * 0.01)
    };

    const newBatteryLevel = Math.max(0, Math.min(100, robot.battery_level - batteryDrain));
    const newTemperature = Math.max(20, Math.min(50, robot.temperature + tempChange));

    // Create motor status with random values
    const motorStatus = {
      motor1: { rpm: Math.floor(Math.random() * 5000), temp: 25 + Math.random() * 15 },
      motor2: { rpm: Math.floor(Math.random() * 5000), temp: 25 + Math.random() * 15 },
    };

    // Determine if we should generate an error
    const generateError = Math.random() > 0.9;
    const errorCodes = generateError 
      ? ["ERR_" + Math.floor(Math.random() * 1000).toString().padStart(3, "0")] 
      : [];

    // Create telemetry record
    const { data: telemetry, error: telemetryError } = await supabaseClient
      .from("telemetry")
      .insert([{
        robot_id: robotId,
        battery_level: newBatteryLevel,
        temperature: newTemperature,
        location: newLocation,
        motor_status: motorStatus,
        error_codes: errorCodes
      }]);

    if (telemetryError) {
      return new Response(
        JSON.stringify({ error: "Failed to insert telemetry", details: telemetryError }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 500 }
      );
    }

    // Create alert if needed
    if (newBatteryLevel < 20 || newTemperature > 40 || generateError) {
      let alertMessage = "";
      let alertType = "";
      
      if (newBatteryLevel < 20) {
        alertType = "battery";
        alertMessage = `Low battery alert: ${newBatteryLevel.toFixed(1)}%`;
      } else if (newTemperature > 40) {
        alertType = "temperature";
        alertMessage = `High temperature alert: ${newTemperature.toFixed(1)}°C`;
      } else if (generateError) {
        alertType = "error";
        alertMessage = `Error detected: ${errorCodes.join(", ")}`;
      }

      await supabaseClient.from("alerts").insert([{
        robot_id: robotId,
        type: alertType,
        message: alertMessage,
      }]);
    }

    return new Response(
      JSON.stringify({ 
        success: true, 
        robot_id: robotId,
        battery_level: newBatteryLevel,
        temperature: newTemperature
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
