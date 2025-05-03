
import { serve } from "https://deno.land/std@0.190.0/http/server.ts";
import Stripe from "https://esm.sh/stripe@14.10.0?target=deno";
import { createClient } from "https://esm.sh/@supabase/supabase-js@2.38.4";

const corsHeaders = {
  "Access-Control-Allow-Origin": "*",
  "Access-Control-Allow-Headers":
    "authorization, x-client-info, apikey, content-type",
};

serve(async (req) => {
  // Handle CORS
  if (req.method === "OPTIONS") {
    return new Response(null, {
      headers: corsHeaders,
    });
  }

  try {
    // Get the authorization header
    const authHeader = req.headers.get("Authorization")!;
    if (!authHeader) {
      throw new Error("No authorization header");
    }

    // Create Supabase client
    const supabaseUrl = Deno.env.get("SUPABASE_URL")!;
    const supabaseKey = Deno.env.get("SUPABASE_SERVICE_ROLE_KEY")!;
    const supabase = createClient(supabaseUrl, supabaseKey);

    // Get user from authorization header
    const token = authHeader.replace("Bearer ", "");
    const { data: { user }, error: userError } = await supabase.auth.getUser(token);
    if (userError || !user) {
      throw new Error("Error getting user: " + (userError?.message || "User not found"));
    }

    // Parse the request body
    const { priceId } = await req.json();
    if (!priceId) {
      throw new Error("Price ID is required");
    }

    // Get the actual Stripe price ID from the environment variable
    const stripePriceId = Deno.env.get(priceId);
    if (!stripePriceId) {
      throw new Error(`Price ID environment variable ${priceId} not found`);
    }

    // Create Stripe instance
    const stripe = new Stripe(Deno.env.get("STRIPE_SECRET_KEY")!, {
      apiVersion: "2023-10-16",
    });

    // Check if customer exists in Stripe
    let customerId;
    const customers = await stripe.customers.list({ email: user.email, limit: 1 });
    
    if (customers.data.length > 0) {
      customerId = customers.data[0].id;
    } else {
      // Create a new customer in Stripe
      const customer = await stripe.customers.create({
        email: user.email,
        metadata: {
          user_id: user.id,
        }
      });
      customerId = customer.id;
    }

    // Get site URL from environment or request
    const siteUrl = Deno.env.get("SITE_URL") || req.headers.get("origin") || "http://localhost:5173";

    // Get plan name based on priceId
    let planName = "Custom";
    if (priceId === "STRIPE_STARTER_PRICE_ID") {
      planName = "Starter";
    } else if (priceId === "STRIPE_GROWTH_PRICE_ID") {
      planName = "Growth";
    } else if (priceId === "STRIPE_PRO_PRICE_ID") {
      planName = "Pro";
    }

    // Check if the user already has a subscription record, if not create one
    const { data: existingSubscription } = await supabase
      .from("subscriptions")
      .select("*")
      .eq("user_id", user.id)
      .single();

    if (!existingSubscription) {
      // Create an initial subscription record (will be updated when payment completes)
      await supabase.from("subscriptions").insert({
        user_id: user.id,
        stripe_customer_id: customerId,
        status: 'incomplete',
        plan_id: stripePriceId,
        plan_name: planName,
        created_at: new Date().toISOString(),
        updated_at: new Date().toISOString(),
      });
    }

    // Create a checkout session with Stripe
    const session = await stripe.checkout.sessions.create({
      customer: customerId,
      mode: "subscription",
      payment_method_types: ["card"],
      line_items: [
        {
          price: stripePriceId,
          quantity: 1,
        },
      ],
      success_url: `${siteUrl}/dashboard?payment=success`,
      cancel_url: `${siteUrl}/pricing?payment=canceled`,
      subscription_data: {
        metadata: {
          user_id: user.id,
          plan_name: planName
        },
      },
    });

    return new Response(JSON.stringify({ url: session.url }), {
      headers: { ...corsHeaders, "Content-Type": "application/json" },
      status: 200,
    });
  } catch (error) {
    console.error("Error:", error.message);
    return new Response(
      JSON.stringify({ error: error.message }),
      {
        headers: { ...corsHeaders, "Content-Type": "application/json" },
        status: 400,
      }
    );
  }
});
