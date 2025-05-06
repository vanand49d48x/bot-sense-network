
import { serve } from "https://deno.land/std@0.177.0/http/server.ts";
import Stripe from "https://esm.sh/stripe@12.0.0";

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
    const { planId, userId } = await req.json();
    
    if (!planId) {
      throw new Error("Plan ID is required");
    }
    
    // Initialize Stripe with the secret key from environment variables
    const stripe = new Stripe(Deno.env.get("STRIPE_SECRET_KEY") || "", {
      apiVersion: "2023-10-16",
    });
    
    // Get price ID based on plan
    let priceId;
    switch (planId) {
      case "starter":
        priceId = Deno.env.get("STRIPE_STARTER_PRICE_ID");
        break;
      case "growth":
        priceId = Deno.env.get("STRIPE_GROWTH_PRICE_ID");
        break;
      case "pro":
        priceId = Deno.env.get("STRIPE_PRO_PRICE_ID");
        break;
      default:
        throw new Error("Invalid plan selected");
    }
    
    if (!priceId) {
      throw new Error(`Price ID not found for plan: ${planId}`);
    }
    
    // Create a Stripe checkout session
    const session = await stripe.checkout.sessions.create({
      payment_method_types: ["card"],
      line_items: [
        {
          price: priceId,
          quantity: 1,
        },
      ],
      mode: "subscription",
      success_url: `${req.headers.get("origin")}/subscription-success?session_id={CHECKOUT_SESSION_ID}`,
      cancel_url: `${req.headers.get("origin")}/pricing?canceled=true`,
      client_reference_id: userId,
      // Pass customer email if available (for logged in users)
      ...(req.headers.get("X-User-Email") ? { customer_email: req.headers.get("X-User-Email") } : {}),
    });
    
    return new Response(
      JSON.stringify({ 
        sessionId: session.id, 
        url: session.url 
      }),
      { 
        headers: { 
          ...corsHeaders,
          "Content-Type": "application/json" 
        }
      }
    );
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
