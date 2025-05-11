import { serve } from "https://deno.land/std@0.168.0/http/server.ts";
import { createClient } from "https://esm.sh/@supabase/supabase-js@2.43.0";
import { v4 as uuidv4 } from "https://esm.sh/uuid@9.0.0";

const corsHeaders = {
  "Access-Control-Allow-Origin": "*",
  "Access-Control-Allow-Headers": "authorization, x-client-info, apikey, content-type",
};

// Determine the correct Supabase URL and key based on the active environment
// For edge functions, we use environment variables set during deployment
const SUPABASE_URL = Deno.env.get("SUPABASE_URL") ?? "";
const SUPABASE_SERVICE_ROLE_KEY = Deno.env.get("SUPABASE_SERVICE_ROLE_KEY") ?? "";

serve(async (req) => {
  // Handle CORS preflight requests
  if (req.method === "OPTIONS") {
    return new Response(null, { headers: corsHeaders });
  }

  try {
    const supabaseClient = createClient(
      SUPABASE_URL,
      SUPABASE_SERVICE_ROLE_KEY
    );

    // Extract auth token from request header
    const authHeader = req.headers.get('Authorization');
    if (!authHeader) {
      return new Response(
        JSON.stringify({ error: "Authorization header is required" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
      );
    }

    // Extract the token from the Authorization header
    const token = authHeader.replace('Bearer ', '');
    
    // Get the user from the token
    const { data: { user }, error: authError } = await supabaseClient.auth.getUser(token);
    
    if (authError || !user) {
      return new Response(
        JSON.stringify({ error: "Invalid authentication token", details: authError }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
      );
    }

    if (req.method !== "POST") {
      return new Response(
        JSON.stringify({ error: "Method not allowed" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 405 }
      );
    }

    // Parse request body
    const { name, type, description } = await req.json();

    if (!name || !type) {
      return new Response(
        JSON.stringify({ error: "Name and type are required" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 400 }
      );
    }
    
    // Generate a unique API key
    const apiKey = uuidv4();

    // Create robot with API key
    const { data, error } = await supabaseClient
      .from("robots")
      .insert({
        name,
        type,
        description,
        user_id: user.id,
        api_key: apiKey
      })
      .select("*")
      .single();

    if (error) {
      return new Response(
        JSON.stringify({ error: "Failed to create robot", details: error }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 500 }
      );
    }

    // Return the robot data along with the API key
    return new Response(
      JSON.stringify({ 
        success: true, 
        robot: {
          ...data,
          api_key: apiKey // Include the API key in the response
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
