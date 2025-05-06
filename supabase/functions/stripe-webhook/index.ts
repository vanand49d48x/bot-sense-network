
import { serve } from "https://deno.land/std@0.177.0/http/server.ts";
import Stripe from "https://esm.sh/stripe@12.0.0";
import { createClient } from "https://esm.sh/@supabase/supabase-js@2.7.1";

const corsHeaders = {
  "Access-Control-Allow-Origin": "*",
  "Access-Control-Allow-Headers": "authorization, x-client-info, apikey, content-type",
};

// This webhook doesn't require JWT verification
serve(async (req) => {
  // Handle CORS preflight requests
  if (req.method === "OPTIONS") {
    return new Response(null, { headers: corsHeaders });
  }
  
  try {
    const supabaseUrl = Deno.env.get("SUPABASE_URL");
    const supabaseServiceKey = Deno.env.get("SUPABASE_SERVICE_ROLE_KEY");
    
    if (!supabaseUrl || !supabaseServiceKey) {
      throw new Error("Supabase environment variables not configured correctly");
    }
    
    // Create a Supabase client with the service role key
    const supabase = createClient(supabaseUrl, supabaseServiceKey, {
      auth: {
        persistSession: false
      }
    });
    
    // Get the stripe signature from the headers
    const signature = req.headers.get("stripe-signature");
    
    if (!signature) {
      throw new Error("No stripe signature found in the request");
    }
    
    // Get the raw request body
    const body = await req.text();
    
    // Initialize Stripe with the secret key
    const stripe = new Stripe(Deno.env.get("STRIPE_SECRET_KEY") || "", {
      apiVersion: "2023-10-16",
    });
    
    // Verify the webhook
    const stripeWebhookSecret = Deno.env.get("STRIPE_WEBHOOK_SECRET");
    if (!stripeWebhookSecret) {
      throw new Error("Stripe webhook secret not configured");
    }
    
    const event = stripe.webhooks.constructEvent(
      body,
      signature,
      stripeWebhookSecret
    );
    
    // Handle different event types
    switch (event.type) {
      case "checkout.session.completed": {
        const session = event.data.object;
        
        // If there's a client_reference_id, it contains the user ID
        if (session.client_reference_id) {
          const userId = session.client_reference_id;
          const customerId = session.customer;
          const subscriptionId = session.subscription;
          
          if (typeof customerId !== "string" || typeof subscriptionId !== "string") {
            throw new Error("Customer ID or Subscription ID not found in the session");
          }
          
          // Get subscription details
          const subscription = await stripe.subscriptions.retrieve(subscriptionId);
          const priceId = subscription.items.data[0].price.id;
          
          // Determine the plan name based on price ID
          let planName = "Unknown";
          let planId = "unknown";
          
          if (priceId === Deno.env.get("STRIPE_STARTER_PRICE_ID")) {
            planName = "Starter";
            planId = "starter";
          } else if (priceId === Deno.env.get("STRIPE_GROWTH_PRICE_ID")) {
            planName = "Growth";
            planId = "growth";
          } else if (priceId === Deno.env.get("STRIPE_PRO_PRICE_ID")) {
            planName = "Pro";
            planId = "pro";
          }
          
          // Update the subscription in the database
          await supabase
            .from("subscriptions")
            .upsert({
              user_id: userId,
              stripe_customer_id: customerId,
              stripe_subscription_id: subscriptionId,
              plan_id: planId,
              plan_name: planName,
              status: subscription.status,
              current_period_end: new Date(subscription.current_period_end * 1000).toISOString(),
              updated_at: new Date().toISOString()
            });
        }
        break;
      }
      
      case "customer.subscription.updated": {
        const subscription = event.data.object;
        
        // Find the user with this subscription
        const { data } = await supabase
          .from("subscriptions")
          .select("*")
          .eq("stripe_subscription_id", subscription.id)
          .maybeSingle();
        
        if (data) {
          // Update subscription status
          await supabase
            .from("subscriptions")
            .update({
              status: subscription.status,
              current_period_end: new Date(subscription.current_period_end * 1000).toISOString(),
              updated_at: new Date().toISOString()
            })
            .eq("id", data.id);
        }
        break;
      }
      
      case "customer.subscription.deleted": {
        const subscription = event.data.object;
        
        // Find the user with this subscription
        const { data } = await supabase
          .from("subscriptions")
          .select("*")
          .eq("stripe_subscription_id", subscription.id)
          .maybeSingle();
        
        if (data) {
          // Update subscription status to canceled
          await supabase
            .from("subscriptions")
            .update({
              status: "canceled",
              updated_at: new Date().toISOString()
            })
            .eq("id", data.id);
        }
        break;
      }
    }
    
    return new Response(JSON.stringify({ received: true }), {
      headers: { ...corsHeaders, "Content-Type": "application/json" }
    });
  } catch (error) {
    return new Response(
      JSON.stringify({ error: error.message }),
      { 
        status: 400, 
        headers: { 
          ...corsHeaders,
          "Content-Type": "application/json" 
        }
      }
    );
  }
});
