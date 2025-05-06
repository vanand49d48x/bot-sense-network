
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
    // Get the request body and extract the priceId
    const { priceId } = await req.json();
    
    if (!priceId) {
      throw new Error("No price ID provided");
    }
    
    // Get the price ID from environment variables based on the provided price ID
    const priceLookup: Record<string, string> = {
      "STRIPE_STARTER_PRICE_ID": Deno.env.get("STRIPE_STARTER_PRICE_ID") || "",
      "STRIPE_GROWTH_PRICE_ID": Deno.env.get("STRIPE_GROWTH_PRICE_ID") || "",
      "STRIPE_PRO_PRICE_ID": Deno.env.get("STRIPE_PRO_PRICE_ID") || "",
    };
    
    const actualPriceId = priceLookup[priceId] || priceId;
    
    if (!actualPriceId) {
      throw new Error(`Invalid price ID: ${priceId}`);
    }
    
    console.log(`Using price ID: ${actualPriceId}`);

    // Get the authorization header
    const authHeader = req.headers.get("Authorization");
    if (!authHeader) {
      throw new Error("No authorization header provided");
    }

    // Get Stripe key from environment
    const stripeKey = Deno.env.get("STRIPE_SECRET_KEY");
    if (!stripeKey) {
      throw new Error("Stripe key not set in environment");
    }

    // Initialize Supabase client
    const supabaseUrl = Deno.env.get("SUPABASE_URL") || "";
    const supabaseServiceKey = Deno.env.get("SUPABASE_SERVICE_ROLE_KEY") || "";
    const supabaseClient = createClient(supabaseUrl, supabaseServiceKey, {
      auth: {
        persistSession: false,
      },
    });

    // Authenticate the user with the provided token
    const token = authHeader.replace("Bearer ", "");
    const { data: userData, error: userError } = await supabaseClient.auth.getUser(token);
    
    if (userError || !userData.user) {
      throw new Error("User not authenticated");
    }

    const user = userData.user;
    console.log(`User authenticated: ${user.id} ${user.email}`);

    // Initialize Stripe
    const stripe = new Stripe(stripeKey, {
      apiVersion: "2023-10-16",
    });

    // Check if the user already has a customer account in Stripe
    const customers = await stripe.customers.list({
      email: user.email,
      limit: 1,
    });

    let customerId: string;
    if (customers.data.length > 0) {
      customerId = customers.data[0].id;
      console.log(`Existing customer found: ${customerId}`);
    } else {
      // Create a new customer if one doesn't exist
      const newCustomer = await stripe.customers.create({
        email: user.email,
        metadata: {
          supabase_uid: user.id,
        },
      });
      customerId = newCustomer.id;
      console.log(`New customer created: ${customerId}`);
    }

    // Get the origin of the request
    const origin = req.headers.get("origin") || "http://localhost:3000";

    // Create a Stripe checkout session
    const session = await stripe.checkout.sessions.create({
      customer: customerId,
      payment_method_types: ["card"],
      line_items: [
        {
          price: actualPriceId,
          quantity: 1,
        },
      ],
      mode: "subscription",
      success_url: `${origin}/checkout-success?session_id=${session?.id}`,
      cancel_url: `${origin}/checkout-cancel`,
      allow_promotion_codes: true,
    });

    console.log(`Checkout session created: ${session.id}`);

    return new Response(
      JSON.stringify({ url: session.url }),
      {
        headers: { ...corsHeaders, "Content-Type": "application/json" },
        status: 200,
      }
    );
  } catch (error) {
    console.error(`Error in create-checkout: ${error.message}`);
    
    return new Response(
      JSON.stringify({ error: error.message }),
      {
        headers: { ...corsHeaders, "Content-Type": "application/json" },
        status: 500,
      }
    );
  }
});
