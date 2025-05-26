
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
    logStep("User authenticated", {
      userId: user.id,
      email: user.email
    });

    // FIRST: Check database for existing subscription
    const { data: dbSubscription, error: dbError } = await supabaseClient
      .from("subscriptions")
      .select("*")
      .eq("user_id", user.id)
      .order("created_at", { ascending: false })
      .limit(1)
      .single();

    logStep("Database subscription check", { 
      found: !!dbSubscription, 
      planName: dbSubscription?.plan_name,
      status: dbSubscription?.status,
      error: dbError?.message 
    });

    // If we have a database subscription, prioritize it over Stripe
    if (dbSubscription && dbSubscription.status === "active") {
      logStep("Found active subscription in database", {
        planName: dbSubscription.plan_name,
        status: dbSubscription.status,
        trialStatus: dbSubscription.trial_status
      });

      // Calculate days remaining for trials
      let daysRemaining = null;
      if (dbSubscription.trial_status === 'active' && dbSubscription.trial_ended_at) {
        const trialEnd = new Date(dbSubscription.trial_ended_at);
        const now = new Date();
        const diffTime = trialEnd.getTime() - now.getTime();
        daysRemaining = Math.ceil(diffTime / (1000 * 60 * 60 * 24));
        if (daysRemaining < 0) {
          daysRemaining = 0;
          // Update trial status to expired if past end date
          await supabaseClient
            .from("subscriptions")
            .update({ 
              trial_status: 'expired',
              status: 'inactive',
              updated_at: new Date().toISOString()
            })
            .eq("id", dbSubscription.id);
          
          logStep("Trial expired, updated status");
          
          return new Response(
            JSON.stringify({
              active: false,
              plan: dbSubscription.plan_name,
              trial_status: 'expired',
              subscription_end: dbSubscription.trial_ended_at,
              days_remaining: 0,
            }),
            {
              headers: { ...corsHeaders, "Content-Type": "application/json" },
              status: 200,
            }
          );
        }
      }

      return new Response(
        JSON.stringify({
          active: true,
          plan: dbSubscription.plan_name,
          trial_status: dbSubscription.trial_status,
          subscription_end: dbSubscription.trial_ended_at || dbSubscription.current_period_end,
          days_remaining: daysRemaining,
          subscription_id: dbSubscription.stripe_subscription_id,
        }),
        {
          headers: { ...corsHeaders, "Content-Type": "application/json" },
          status: 200,
        }
      );
    }

    // If no active database subscription, check Stripe
    const stripeKey = Deno.env.get("STRIPE_SECRET_KEY");
    if (!stripeKey) {
      logStep("No Stripe key, returning inactive status");
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

    logStep("Stripe key verified, checking Stripe subscriptions");
    const stripe = new Stripe(stripeKey, { apiVersion: "2023-10-16" });

    // Check if a Stripe customer exists for this user
    const customers = await stripe.customers.list({ email: user.email, limit: 1 });
    
    if (customers.data.length === 0) {
      logStep("No Stripe customer found, updating as unsubscribed");
      
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
      expand: ["data.items.data.price"],
    });

    if (subscriptions.data.length === 0) {
      logStep("No active Stripe subscription found");
      
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

    // We have an active Stripe subscription
    const subscription = subscriptions.data[0];
    const price = subscription.items.data[0].price;
    
    // Get the product details separately
    const product = await stripe.products.retrieve(price.product as string);
    const planName = product.name;
    const subscriptionEnd = new Date(subscription.current_period_end * 1000).toISOString();
    
    // Check if subscription is canceled but still in current period
    const isCanceled = subscription.cancel_at_period_end;
    
    logStep("Active Stripe subscription found", { 
      subscriptionId: subscription.id, 
      planName, 
      endDate: subscriptionEnd,
      isCanceled
    });

    // Update subscription in database
    await supabaseClient.from("subscriptions").upsert({
      user_id: user.id,
      stripe_customer_id: customerId,
      stripe_subscription_id: subscription.id,
      status: isCanceled ? "canceled" : "active",
      plan_id: price.id,
      plan_name: planName,
      current_period_end: subscriptionEnd,
      updated_at: new Date().toISOString(),
    }, { onConflict: 'user_id' });

    logStep("Updated database with Stripe subscription info");
    
    return new Response(
      JSON.stringify({
        active: !isCanceled,
        plan: planName,
        subscription_end: subscriptionEnd,
        subscription_id: subscription.id,
        cancel_at_period_end: subscription.cancel_at_period_end,
        price: {
          id: price.id,
          amount: price.unit_amount,
          currency: price.currency,
        }
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
