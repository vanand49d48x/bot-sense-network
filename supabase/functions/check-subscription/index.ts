
import { serve } from "https://deno.land/std@0.177.0/http/server.ts";
import { createClient } from "https://esm.sh/@supabase/supabase-js@2.26.0";
import Stripe from 'https://esm.sh/stripe@14.21.0';

const corsHeaders = {
  "Access-Control-Allow-Origin": "*",
  "Access-Control-Allow-Headers": "authorization, x-client-info, apikey, content-type",
};

const logStep = (step: string, details?: any) => {
  const detailsStr = details ? ` - ${JSON.stringify(details)}` : '';
  console.log(`[CHECK-SUBSCRIPTION] ${step}${detailsStr}`);
};

serve(async (req) => {
  // Handle CORS preflight requests
  if (req.method === "OPTIONS") {
    return new Response(null, { headers: corsHeaders });
  }

  try {
    logStep("Function started");

    // Get the Stripe secret key from environment
    const stripeKey = Deno.env.get("STRIPE_SECRET_KEY");
    if (!stripeKey) {
      throw new Error("STRIPE_SECRET_KEY not set");
    }
    logStep("Stripe key verified");

    // Get the authorization header
    const authHeader = req.headers.get("Authorization");
    if (!authHeader) {
      throw new Error("No authorization header provided");
    }
    logStep("Authorization header found");

    // Initialize Supabase client with service role key for database writes
    const supabaseClient = createClient(
      Deno.env.get("SUPABASE_URL") || "",
      Deno.env.get("SUPABASE_SERVICE_ROLE_KEY") || "",
      { auth: { persistSession: false } }
    );

    // Authenticate the user with the provided token
    const token = authHeader.replace("Bearer ", "");
    logStep("Authenticating user with token");
    
    const { data: userData, error: userError } = await supabaseClient.auth.getUser(token);
    if (userError || !userData.user) {
      throw new Error("User not authenticated");
    }

    const user = userData.user;
    logStep("User authenticated", { userId: user.id, email: user.email });

    // Initialize Stripe
    const stripe = new Stripe(stripeKey, { apiVersion: "2023-10-16" });

    // Check if a Stripe customer exists for this user
    const customers = await stripe.customers.list({ email: user.email, limit: 1 });
    
    if (customers.data.length === 0) {
      logStep("No customer found, updating as unsubscribed");
      
      // Update subscription record in database as inactive
      await supabaseClient.from("subscriptions").upsert({
        user_id: user.id,
        status: "inactive",
        updated_at: new Date().toISOString(),
      }, { onConflict: 'user_id' });

      return new Response(
        JSON.stringify({ 
          active: false, 
          plan: null,
          subscription_end: null
        }),
        {
          headers: { ...corsHeaders, "Content-Type": "application/json" },
          status: 200,
        }
      );
    }

    const customerId = customers.data[0].id;
    logStep("Found Stripe customer", { customerId });

    // Check for active subscriptions
    const subscriptions = await stripe.subscriptions.list({
      customer: customerId,
      status: "active",
      expand: ["data.items.data.price.product"],
    });

    if (subscriptions.data.length === 0) {
      logStep("No active subscription found");
      
      // Update subscription record in database as inactive
      await supabaseClient.from("subscriptions").upsert({
        user_id: user.id,
        status: "inactive",
        stripe_customer_id: customerId,
        updated_at: new Date().toISOString(),
      }, { onConflict: 'user_id' });

      return new Response(
        JSON.stringify({ 
          active: false, 
          plan: null,
          subscription_end: null
        }),
        {
          headers: { ...corsHeaders, "Content-Type": "application/json" },
          status: 200,
        }
      );
    }

    // We have an active subscription
    const subscription = subscriptions.data[0];
    const price = subscription.items.data[0].price;
    const product = price.product as Stripe.Product;
    
    const planName = product.name;
    const subscriptionEnd = new Date(subscription.current_period_end * 1000).toISOString();
    
    logStep("Active subscription found", { 
      subscriptionId: subscription.id, 
      planName, 
      endDate: subscriptionEnd 
    });

    // Update subscription in database
    await supabaseClient.from("subscriptions").upsert({
      user_id: user.id,
      stripe_customer_id: customerId,
      stripe_subscription_id: subscription.id,
      status: "active",
      plan_id: price.id,
      plan_name: planName,
      current_period_end: subscriptionEnd,
      updated_at: new Date().toISOString(),
    }, { onConflict: 'user_id' });

    logStep("Updated database with subscription info");
    
    return new Response(
      JSON.stringify({
        active: true,
        plan: planName,
        subscription_end: subscriptionEnd,
        subscription_id: subscription.id,
      }),
      {
        headers: { ...corsHeaders, "Content-Type": "application/json" },
        status: 200,
      }
    );
  } catch (error) {
    logStep("ERROR", { message: error.message });
    return new Response(
      JSON.stringify({ error: error.message }),
      {
        headers: { ...corsHeaders, "Content-Type": "application/json" },
        status: 500,
      }
    );
  }
});
