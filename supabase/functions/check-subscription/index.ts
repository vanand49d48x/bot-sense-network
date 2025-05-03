
import { serve } from "https://deno.land/std@0.190.0/http/server.ts";
import Stripe from "https://esm.sh/stripe@14.10.0?target=deno";
import { createClient } from "https://esm.sh/@supabase/supabase-js@2.38.4";

const corsHeaders = {
  "Access-Control-Allow-Origin": "*",
  "Access-Control-Allow-Headers":
    "authorization, x-client-info, apikey, content-type",
};

// Helper logging function for enhanced debugging
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

    // Use the service role key to perform writes (upsert) in Supabase
    const supabaseUrl = Deno.env.get("SUPABASE_URL")!;
    const supabaseServiceKey = Deno.env.get("SUPABASE_SERVICE_ROLE_KEY")!;
    const supabaseClient = createClient(supabaseUrl, supabaseServiceKey);

    const stripeKey = Deno.env.get("STRIPE_SECRET_KEY");
    if (!stripeKey) throw new Error("STRIPE_SECRET_KEY is not set");
    logStep("Stripe key verified");

    const authHeader = req.headers.get("Authorization");
    if (!authHeader) throw new Error("No authorization header provided");
    logStep("Authorization header found");

    const token = authHeader.replace("Bearer ", "");
    logStep("Authenticating user with token");
    
    const { data: userData, error: userError } = await supabaseClient.auth.getUser(token);
    if (userError) throw new Error(`Authentication error: ${userError.message}`);
    const user = userData.user;
    if (!user?.email) throw new Error("User not authenticated or email not available");
    logStep("User authenticated", { userId: user.id, email: user.email });

    const stripe = new Stripe(stripeKey, { apiVersion: "2023-10-16" });
    const customers = await stripe.customers.list({ email: user.email, limit: 1 });
    
    if (customers.data.length === 0) {
      logStep("No customer found, updating unsubscribed state");
      await supabaseClient.from("subscriptions").upsert({
        user_id: user.id,
        stripe_customer_id: null,
        status: 'inactive',
        plan_name: null,
        current_period_end: null,
        updated_at: new Date().toISOString(),
      }, { onConflict: 'user_id' });
      return new Response(JSON.stringify({ 
        subscribed: false,
        plan: null,
        current_period_end: null
      }), {
        headers: { ...corsHeaders, "Content-Type": "application/json" },
        status: 200,
      });
    }

    const customerId = customers.data[0].id;
    logStep("Found Stripe customer", { customerId });

    const subscriptions = await stripe.subscriptions.list({
      customer: customerId,
      status: "active",
      limit: 1,
    });
    const hasActiveSub = subscriptions.data.length > 0;
    let planName = null;
    let currentPeriodEnd = null;
    let priceId = null;
    
    if (hasActiveSub) {
      const subscription = subscriptions.data[0];
      currentPeriodEnd = new Date(subscription.current_period_end * 1000).toISOString();
      const stripeSubscriptionId = subscription.id;
      logStep("Active subscription found", { subscriptionId: stripeSubscriptionId, endDate: currentPeriodEnd });
      
      // Get the price ID from subscription
      priceId = subscription.items.data[0].price.id;
      
      // Match price ID with plan name
      if (priceId === Deno.env.get("STRIPE_STARTER_PRICE_ID")) {
        planName = "Starter";
      } else if (priceId === Deno.env.get("STRIPE_GROWTH_PRICE_ID")) {
        planName = "Growth";
      } else if (priceId === Deno.env.get("STRIPE_PRO_PRICE_ID")) {
        planName = "Pro";
      } else {
        planName = "Custom";
      }
      
      logStep("Determined plan name", { priceId, planName });

      // Update subscription in database
      await supabaseClient.from("subscriptions").upsert({
        user_id: user.id,
        stripe_customer_id: customerId,
        stripe_subscription_id: stripeSubscriptionId,
        plan_id: priceId,
        plan_name: planName,
        status: 'active',
        current_period_end: currentPeriodEnd,
        updated_at: new Date().toISOString(),
      }, { onConflict: 'user_id' });
      
      logStep("Updated database with subscription info");
    } else {
      logStep("No active subscription found");
      
      // Update as inactive in database
      await supabaseClient.from("subscriptions").upsert({
        user_id: user.id,
        stripe_customer_id: customerId,
        status: 'inactive',
        plan_name: null,
        plan_id: null,
        current_period_end: null,
        updated_at: new Date().toISOString(),
      }, { onConflict: 'user_id' });
    }

    return new Response(JSON.stringify({
      subscribed: hasActiveSub,
      plan: planName,
      current_period_end: currentPeriodEnd,
    }), {
      headers: { ...corsHeaders, "Content-Type": "application/json" },
      status: 200,
    });
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : String(error);
    logStep("ERROR", { message: errorMessage });
    return new Response(JSON.stringify({ error: errorMessage }), {
      headers: { ...corsHeaders, "Content-Type": "application/json" },
      status: 500,
    });
  }
});
