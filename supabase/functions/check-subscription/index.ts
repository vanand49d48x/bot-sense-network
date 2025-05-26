
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

    // Check for subscription in DB first (including Free Tier)
    const { data: dbSub, error: dbSubError } = await supabaseClient
      .from("subscriptions")
      .select("*")
      .eq("user_id", user.id)
      .order("created_at", { ascending: false })
      .limit(1);

    logStep("Database subscription query result", { 
      data: dbSub, 
      error: dbSubError,
      userIdFromToken: user.id 
    });

    if (dbSubError) {
      logStep("Error fetching subscription from database", dbSubError);
    }

    // If we have a subscription record in the database
    if (dbSub && dbSub.length > 0) {
      const subscription = dbSub[0];
      logStep("Found subscription in database", subscription);

      // Handle Free Tier subscription
      if (subscription.plan_name === "Free Tier") {
        const now = new Date();
        const trialEndDate = subscription.trial_ended_at ? new Date(subscription.trial_ended_at) : null;
        
        let trialStatus = subscription.trial_status;
        let isActive = subscription.status === "active";
        
        // Check if trial has expired
        if (trialEndDate && now > trialEndDate && trialStatus === "active") {
          trialStatus = "expired";
          isActive = false;
          
          // Update the database to reflect expired status
          await supabaseClient
            .from("subscriptions")
            .update({ 
              trial_status: "expired", 
              status: "inactive",
              updated_at: new Date().toISOString() 
            })
            .eq("id", subscription.id);
            
          logStep("Updated expired Free Tier subscription");
        }
        
        // Calculate days remaining for active trials
        let daysRemaining = null;
        if (trialStatus === "active" && trialEndDate) {
          const timeDiff = trialEndDate.getTime() - now.getTime();
          daysRemaining = Math.max(0, Math.ceil(timeDiff / (1000 * 3600 * 24)));
        }
        
        logStep("Returning Free Tier subscription", {
          active: isActive,
          trial_status: trialStatus,
          days_remaining: daysRemaining
        });

        return new Response(
          JSON.stringify({
            active: isActive,
            plan: "Free Tier",
            trial_status: trialStatus,
            subscription_end: subscription.trial_ended_at,
            days_remaining: daysRemaining,
          }),
          {
            headers: { ...corsHeaders, "Content-Type": "application/json" },
            status: 200,
          }
        );
      }
    }

    // If no subscription found in database, check Stripe
    const stripeKey = Deno.env.get("STRIPE_SECRET_KEY");
    if (!stripeKey) {
      logStep("No Stripe key found, returning inactive");
      
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

    // Initialize Stripe
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

    // We have an active subscription
    const subscription = subscriptions.data[0];
    const price = subscription.items.data[0].price;
    
    // Get the product details separately
    const product = await stripe.products.retrieve(price.product as string);
    const planName = product.name;
    const subscriptionEnd = new Date(subscription.current_period_end * 1000).toISOString();
    
    // Check if subscription is canceled but still in current period
    const isCanceled = subscription.cancel_at_period_end;
    
    logStep("Active subscription found", { 
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

    logStep("Updated database with subscription info");
    
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
