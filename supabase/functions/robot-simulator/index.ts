
// Import necessary Deno modules for edge function environment
import { serve } from "https://deno.land/std@0.168.0/http/server.ts";
import { createClient } from "https://esm.sh/@supabase/supabase-js@2.43.0";

const corsHeaders = {
  "Access-Control-Allow-Origin": "*",
  "Access-Control-Allow-Headers": "authorization, x-client-info, apikey, api-key, content-type",
};

// Function to generate random telemetry data
function generateRandomTelemetry(robotId) {
  return {
    robotId,
    batteryLevel: Math.floor(Math.random() * 100),
    temperature: parseFloat((20 + Math.random() * 10).toFixed(1)),
    status: randomStatus(),
    location: {
      latitude: 33.7756 + (Math.random() - 0.5) * 0.01,  // Changed from lat to latitude
      longitude: -84.3963 + (Math.random() - 0.5) * 0.01 // Changed from lng to longitude
    },
    timestamp: new Date().toISOString()
  };
}

function randomStatus() {
  const statuses = ["OK", "WARNING", "ERROR"];
  // Weighted distribution - mostly OK, sometimes WARNING, rarely ERROR
  const weights = [0.7, 0.25, 0.05];
  const random = Math.random();
  let weightSum = 0;
  
  for (let i = 0; i < weights.length; i++) {
    weightSum += weights[i];
    if (random <= weightSum) return statuses[i];
  }
  
  return statuses[0]; // Default to OK
}

// Function to send telemetry data for a robot
async function sendTelemetry(robotId, apiKey) {
  const data = generateRandomTelemetry(robotId);
  
  try {
    const response = await fetch(
      "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry", 
      {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          "api-key": apiKey // Changed to match what the telemetry function expects
        },
        body: JSON.stringify(data)
      }
    );
    
    if (!response.ok) {
      const errorText = await response.text();
      console.error(`Error sending telemetry for robot ${robotId}: ${response.status} ${response.statusText}`, errorText);
      return { success: false, error: errorText };
    }
    
    const result = await response.json();
    console.log(`Successfully sent telemetry for robot ${robotId}`, result);
    return { success: true, data: result };
  } catch (error) {
    console.error(`Exception sending telemetry for robot ${robotId}:`, error);
    return { success: false, error: error.message };
  }
}

// Main handler function for the edge function
serve(async (req) => {
  // Handle CORS preflight requests
  if (req.method === "OPTIONS") {
    return new Response(null, { headers: corsHeaders });
  }
  
  try {
    // Extract request data
    const { robotIds, apiKey, intervalSeconds = 10, runCount = 1 } = await req.json();
    
    if (!robotIds || !Array.isArray(robotIds) || robotIds.length === 0) {
      return new Response(
        JSON.stringify({ error: "robotIds array is required" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 400 }
      );
    }
    
    if (!apiKey) {
      return new Response(
        JSON.stringify({ error: "apiKey is required" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 400 }
      );
    }
    
    // Initialize Supabase client to validate robot IDs
    const supabase = createClient(
      Deno.env.get("SUPABASE_URL") ?? "",
      Deno.env.get("SUPABASE_SERVICE_ROLE_KEY") ?? ""
    );
    
    // Verify all robot IDs exist
    const { data: robots, error: robotError } = await supabase
      .from("robots")
      .select("id")
      .in("id", robotIds);
      
    if (robotError) {
      return new Response(
        JSON.stringify({ error: "Failed to verify robot IDs", details: robotError }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 500 }
      );
    }
    
    const foundRobotIds = robots.map(r => r.id);
    const invalidRobotIds = robotIds.filter(id => !foundRobotIds.includes(id));
    
    if (invalidRobotIds.length > 0) {
      return new Response(
        JSON.stringify({ error: "Some robot IDs are invalid", invalidIds: invalidRobotIds }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 400 }
      );
    }
    
    // Send telemetry for each robot immediately
    const results = await Promise.all(robotIds.map(id => sendTelemetry(id, apiKey)));
    
    // For runCount > 1, we'll start a background task to continue sending data
    if (runCount > 1) {
      const maxRuns = Math.min(runCount, 10); // Limit to 10 runs for safety
      
      // Use Edge Runtime waitUntil to run a background task
      // This allows the function to respond immediately while continuing work
      EdgeRuntime.waitUntil((async () => {
        console.log(`Starting background task for ${robotIds.length} robots, ${maxRuns} runs each`);
        
        for (let i = 1; i < maxRuns; i++) {
          await new Promise(resolve => setTimeout(resolve, intervalSeconds * 1000));
          await Promise.all(robotIds.map(id => sendTelemetry(id, apiKey)));
          console.log(`Completed run ${i + 1} of ${maxRuns}`);
        }
        
        console.log("Background telemetry simulation completed");
      })());
    }
    
    return new Response(
      JSON.stringify({ 
        success: true,
        message: `Sent telemetry for ${robotIds.length} robots`,
        results: results,
        simulating: runCount > 1 ? `Continuing to send ${Math.min(runCount, 10) - 1} more times` : null
      }),
      { headers: { ...corsHeaders, "Content-Type": "application/json" } }
    );
  } catch (error) {
    console.error("Error in robot simulator function:", error);
    return new Response(
      JSON.stringify({ error: error.message }),
      { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 500 }
    );
  }
});
