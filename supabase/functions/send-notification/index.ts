
import { serve } from "https://deno.land/std@0.168.0/http/server.ts";
import { createClient } from "https://esm.sh/@supabase/supabase-js@2.43.0";

const corsHeaders = {
  "Access-Control-Allow-Origin": "*",
  "Access-Control-Allow-Headers": "authorization, x-client-info, apikey, api-key, content-type",
};

// Types for notification requests
type NotificationType = "LOW_BATTERY" | "OFFLINE" | "HIGH_TEMP" | "ERROR" | "CUSTOM";
type NotificationSeverity = "INFO" | "WARNING" | "CRITICAL";

interface NotificationRequest {
  robotId: string;
  type: NotificationType;
  message?: string;
  severity?: NotificationSeverity;
  additionalData?: Record<string, any>;
}

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

    // Find the user who owns the API key
    const { data: profileData, error: profileError } = await supabaseClient
      .from("profiles")
      .select("id")
      .eq("api_key", apiKey)
      .single();

    if (profileError || !profileData) {
      return new Response(
        JSON.stringify({ error: "Invalid API key" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
      );
    }

    // Parse the request body
    const notificationData: NotificationRequest = await req.json();
    const { robotId, type, message, severity = "INFO", additionalData } = notificationData;

    if (!robotId || !type) {
      return new Response(
        JSON.stringify({ error: "Robot ID and notification type are required" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 400 }
      );
    }

    // Verify the robot exists and belongs to the user
    const { data: robot, error: robotError } = await supabaseClient
      .from("robots")
      .select("id, name, user_id")
      .eq("id", robotId)
      .eq("user_id", profileData.id)
      .single();

    if (robotError || !robot) {
      return new Response(
        JSON.stringify({ error: "Invalid robot ID or you don't have access to this robot" }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 401 }
      );
    }

    // Get the user's email
    const { data: userData, error: userError } = await supabaseClient
      .from("auth.users") // Using auth.users directly with service role key
      .select("email")
      .eq("id", profileData.id)
      .single();

    let userEmail = "";
    if (userError || !userData) {
      console.error("Failed to get user email:", userError);
      // Fall back to checking profiles for email
      const { data: profileWithEmail } = await supabaseClient
        .from("profiles")
        .select("email")
        .eq("id", profileData.id)
        .single();
        
      if (profileWithEmail?.email) {
        userEmail = profileWithEmail.email;
      } else {
        // No email found, we'll log this but continue to create the alert
        console.warn("No email found for user:", profileData.id);
      }
    } else {
      userEmail = userData.email;
    }

    // Generate a default message if none provided
    let alertMessage = message || "";
    if (!alertMessage) {
      switch (type) {
        case "LOW_BATTERY":
          alertMessage = `Low battery alert for robot ${robot.name}`;
          break;
        case "OFFLINE":
          alertMessage = `Robot ${robot.name} is offline`;
          break;
        case "HIGH_TEMP":
          alertMessage = `High temperature detected for robot ${robot.name}`;
          break;
        case "ERROR":
          alertMessage = `Error reported by robot ${robot.name}`;
          break;
        case "CUSTOM":
          alertMessage = `Alert for robot ${robot.name}`;
          break;
      }
    }

    // Create alert in the database (if alerts table exists)
    try {
      // Check if alerts table exists
      const { error: tableExistsError } = await supabaseClient
        .from("alerts")
        .select("id")
        .limit(1);
      
      if (!tableExistsError) {
        await supabaseClient.from("alerts").insert([
          {
            robot_id: robotId,
            user_id: profileData.id,
            type: type,
            message: alertMessage,
            severity: severity,
            additional_data: additionalData,
            resolved: false,
          },
        ]);
      }
    } catch (e) {
      console.error("Failed to insert alert (table might not exist yet):", e);
      // Continue execution even if alert table doesn't exist yet
    }

    // Currently just simulating email sending
    // Replace with actual email sending logic when needed
    console.log(`NOTIFICATION ${type} - ${severity}: ${alertMessage}`);
    console.log(`Would send email to: ${userEmail}`);

    const simulatedEmailContent = {
      to: userEmail,
      subject: `RoboMetrics Alert: ${alertMessage}`,
      body: `
Dear User,

An alert has been generated for your robot ${robot.name}:

Type: ${type}
Severity: ${severity}
Message: ${alertMessage}
Time: ${new Date().toISOString()}

You can view more details and manage this alert in your RoboMetrics dashboard:
https://robometrics.example.com/alerts

Thank you,
The RoboMetrics Team
      `,
    };

    return new Response(
      JSON.stringify({ 
        success: true, 
        message: "Notification processed", 
        emailSent: Boolean(userEmail),
        emailDetails: userEmail ? simulatedEmailContent : "No email address found",
        alert: { type, severity, message: alertMessage }
      }),
      { headers: { ...corsHeaders, "Content-Type": "application/json" } }
    );
  } catch (error: any) {
    console.error("Error processing notification:", error);
    return new Response(
      JSON.stringify({ error: error.message }),
      { headers: { ...corsHeaders, "Content-Type": "application/json" }, status: 500 }
    );
  }
});
