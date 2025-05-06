
import { serve } from "https://deno.land/std@0.177.0/http/server.ts";
import { createClient } from "https://esm.sh/@supabase/supabase-js@2.26.0";
import Stripe from 'https://esm.sh/stripe@14.21.0';

const corsHeaders = {
  "Access-Control-Allow-Origin": "*",
  "Access-Control-Allow-Headers": "authorization, x-client-info, apikey, content-type",
};

serve(async (req) => {
  // Handle CORS preflight requests
  if (req.method === "OPTIONS") {
    return new Response(null, { headers: corsHeaders });
  }

  try {
    const stripe = new Stripe(Deno.env.get("STRIPE_SECRET_KEY") || "", {
      apiVersion: "2023-10-16",
    });

    // Get the authorization header
    const authHeader = req.headers.get("Authorization");
    if (!authHeader) {
      throw new Error("No authorization header provided");
    }

    // Initialize Supabase client
    const supabaseClient = createClient(
      Deno.env.get("SUPABASE_URL") || "",
      Deno.env.get("SUPABASE_ANON_KEY") || "",
    );

    // Authenticate the user with the provided token
    const token = authHeader.replace("Bearer ", "");
    const { data: userData, error: userError } = await supabaseClient.auth.getUser(token);
    if (userError || !userData.user) {
      throw new Error("User not authenticated");
    }

    const user = userData.user;
    console.log("User authenticated:", user.id, user.email);

    // Get request body
    const { priceId } = await req.json();
    if (!priceId) {
      throw new Error("No price ID provided");
    }

    // Map environment variable names based on priceId
    let actualPriceId;
    if (priceId === "STRIPE_STARTER_PRICE_ID") {
      actualPriceId = Deno.env.get("STRIPE_STARTER_PRICE_ID");
    } else if (priceId === "STRIPE_GROWTH_PRICE_ID") {
      actualPriceId = Deno.env.get("STRIPE_GROWTH_PRICE_ID");
    } else if (priceId === "STRIPE_PRO_PRICE_ID") {
      actualPriceId = Deno.env.get("STRIPE_PRO_PRICE_ID");
    } else {
      actualPriceId = priceId; // Use directly if not one of our constants
    }

    if (!actualPriceId) {
      throw new Error(`Price ID not configured: ${priceId}`);
    }

    // Check if a Stripe customer already exists
    const customers = await stripe.customers.list({ email: user.email, limit: 1 });
    let customerId;
    
    if (customers.data.length > 0) {
      customerId = customers.data[0].id;
      console.log("Existing customer found:", customerId);
    } else {
      // Create a new customer
      const newCustomer = await stripe.customers.create({
        email: user.email,
        metadata: {
          supabase_uid: user.id,
        },
      });
      customerId = newCustomer.id;
      console.log("New customer created:", customerId);
    }

    // Create a checkout session with the price
    const session = await stripe.checkout.sessions.create({
      customer: customerId,
      line_items: [
        {
          price: actualPriceId,
          quantity: 1,
        },
      ],
      mode: "subscription",
      success_url: `${req.headers.get("origin")}/checkout-success`,
      cancel_url: `${req.headers.get("origin")}/checkout-cancel`,
      subscription_data: {
        metadata: {
          supabase_uid: user.id,
        },
      },
    });

    console.log("Checkout session created:", session.id);

    return new Response(
      JSON.stringify({ url: session.url }),
      {
        headers: { ...corsHeaders, "Content-Type": "application/json" },
        status: 200,
      }
    );
  } catch (error) {
    console.error("Error in create-checkout:", error.message);
    return new Response(
      JSON.stringify({ error: error.message }),
      {
        headers: { ...corsHeaders, "Content-Type": "application/json" },
        status: 500,
      }
    );
  }
});
