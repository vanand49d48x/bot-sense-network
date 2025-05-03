
import { serve } from "https://deno.land/std@0.190.0/http/server.ts";
import Stripe from "https://esm.sh/stripe@14.10.0?target=deno";
import { createClient } from "https://esm.sh/@supabase/supabase-js@2.38.4";

const corsHeaders = {
  "Access-Control-Allow-Origin": "*",
  "Access-Control-Allow-Headers":
    "authorization, x-client-info, apikey, content-type",
};

// Helper logging function for debugging
const logStep = (step: string, details?: any) => {
  const detailsStr = details ? ` - ${JSON.stringify(details)}` : '';
  console.log(`[CHECK-SUBSCRIPTION] ${step}${detailsStr}`);
};

serve(async (req) => {
  if (req.method === "OPTIONS") {
    return new Response(null, { headers: corsHeaders });
  }

  try {
    logStep("Function started");

    const stripeKey = Deno.env.get("STRIPE_SECRET_KEY");
    if (!stripeKey) throw new Error("STRIPE_SECRET_KEY is not set");
    logStep("Stripe key verified");

    // Initialize Supabase client with the service role key for admin operations
    const supabaseUrl = Deno.env.get("SUPABASE_URL")!;
    const supabaseServiceKey = Deno.env.get("SUPABASE_SERVICE_ROLE_KEY")!;
    const supabaseClient = createClient(supabaseUrl, supabaseServiceKey);

    const authHeader = req.headers.get("Authorization");
    if (!authHeader) throw new Error("No authorization header provided");
    logStep("Authorization header found");

    const token = authHeader.replace("Bearer ", "");
    const { data: userData, error: userError } = await supabaseClient.auth.getUser(token);
    if (userError) throw new Error(`Authentication error: ${userError.message}`);
    const user = userData.user;
    if (!user?.id) throw new Error("User ID not available");
    logStep("User authenticated", { userId: user.id });

    const stripe = new Stripe(stripeKey, { apiVersion: "2023-10-16" });

    // First check if the user already has subscription info in our database
    const { data: subscriptionData, error: subscriptionError } = await supabaseClient
      .from("subscriptions")
      .select("*")
      .eq("user_id", user.id)
      .single();

    if (subscriptionError && subscriptionError.code !== "PGRST116") {
      // If error is not "no rows returned" error, it's a real error
      logStep("Error fetching subscription from database", { error: subscriptionError });
    }

    let stripeCustomerId = subscriptionData?.stripe_customer_id;
    
    // If no customer ID in our database, search in Stripe
    if (!stripeCustomerId && user.email) {
      logStep("No customer ID found in database, checking Stripe");
      const customers = await stripe.customers.list({
        email: user.email,
        limit: 1,
      });
      
      if (customers.data.length > 0) {
        stripeCustomerId = customers.data[0].id;
        logStep("Found Stripe customer by email", { customerId: stripeCustomerId });
      }
    }

    // If still no customer ID, user is not subscribed
    if (!stripeCustomerId) {
      logStep("No Stripe customer found, returning unsubscribed state");
      
      // Update our database
      await supabaseClient.from("subscriptions").upsert({
        user_id: user.id,
        status: "inactive",
        updated_at: new Date().toISOString(),
      });
      
      return new Response(
        JSON.stringify({ subscribed: false, plan: null, current_period_end: null }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" } }
      );
    }

    // Get active subscriptions for customer
    const subscriptions = await stripe.subscriptions.list({
      customer: stripeCustomerId,
      status: "active",
      expand: ["data.items.data.price.product"],
    });
    
    logStep("Stripe subscriptions retrieved", { 
      count: subscriptions.data.length,
      subscriptionIds: subscriptions.data.map(s => s.id)
    });

    if (subscriptions.data.length === 0) {
      // No active subscriptions
      logStep("No active subscriptions found");
      
      // Update our database
      await supabaseClient.from("subscriptions").upsert({
        user_id: user.id,
        stripe_customer_id: stripeCustomerId,
        status: "inactive",
        updated_at: new Date().toISOString(),
      }, { onConflict: "user_id" });
      
      return new Response(
        JSON.stringify({ subscribed: false, plan: null, current_period_end: null }),
        { headers: { ...corsHeaders, "Content-Type": "application/json" } }
      );
    }

    // Get the most recent active subscription
    const subscription = subscriptions.data[0];
    const currentPeriodEnd = new Date(subscription.current_period_end * 1000);
    
    // Get the price ID and product name
    const priceId = subscription.items.data[0].price.id;
    const product = subscription.items.data[0].price.product as Stripe.Product;
    const planName = product.name as "Starter" | "Growth" | "Pro" | "Custom";
    
    logStep("Active subscription found", { 
      priceId, 
      planName, 
      currentPeriodEnd: currentPeriodEnd.toISOString() 
    });

    // Update our database
    await supabaseClient.from("subscriptions").upsert({
      user_id: user.id,
      stripe_customer_id: stripeCustomerId,
      stripe_subscription_id: subscription.id,
      plan_id: priceId,
      plan_name: planName,
      status: "active",
      current_period_end: currentPeriodEnd.toISOString(),
      updated_at: new Date().toISOString(),
    }, { onConflict: "user_id" });

    logStep("Database updated with subscription info");

    return new Response(
      JSON.stringify({
        subscribed: true,
        plan: planName,
        current_period_end: currentPeriodEnd.toISOString(),
      }),
      { headers: { ...corsHeaders, "Content-Type": "application/json" } }
    );
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : String(error);
    logStep("ERROR", { message: errorMessage });
    return new Response(
      JSON.stringify({ error: errorMessage, subscribed: false }),
      {
        headers: { ...corsHeaders, "Content-Type": "application/json" },
        status: 500,
      }
    );
  }
});
