import { serve } from "https://deno.land/std@0.177.0/http/server.ts";
import { createClient } from "https://esm.sh/@supabase/supabase-js@2.26.0";
import Stripe from 'https://esm.sh/stripe@14.21.0';

// This is needed to verify the webhook signature
const endpointSecret = Deno.env.get("STRIPE_WEBHOOK_SECRET") || "";

// Don't add CORS headers for webhooks
// We want strict verification of the origin

// Brevo integration
const BREVO_API_KEY = Deno.env.get("BREVO_API_KEY")!;
const BREVO_TEMPLATE_ID = Number(Deno.env.get("BREVO_TEMPLATE_ID")!);
const BREVO_REACTIVATION_TEMPLATE_ID = 3;

async function sendSubscriptionEmail({
  to,
  userName,
  planName,
  price,
  renewalDate,
  subscriptionId,
  supportEmail,
  endDate,
  templateId
}: {
  to: string,
  userName: string,
  planName: string,
  price?: string,
  renewalDate?: string,
  subscriptionId?: string,
  supportEmail: string,
  endDate?: string,
  templateId?: number
}) {
  const response = await fetch('https://api.brevo.com/v3/smtp/email', {
    method: 'POST',
    headers: {
      'api-key': BREVO_API_KEY,
      'Content-Type': 'application/json',
      'accept': 'application/json'
    },
    body: JSON.stringify({
      to: [{ email: to }],
      templateId: templateId || BREVO_TEMPLATE_ID,
      params: {
        USER_NAME: userName,
        PLAN_NAME: planName,
        PRICE: price,
        RENEWAL_DATE: renewalDate,
        SUBSCRIPTION_ID: subscriptionId,
        SUPPORT_EMAIL: supportEmail,
        END_DATE: endDate
      }
    })
  });
  if (!response.ok) {
    const error = await response.text();
    throw new Error(`Brevo email failed: ${error}`);
  }
}

serve(async (req) => {
  try {
    // Get the stripe signature from the headers
    const sig = req.headers.get("stripe-signature");
    if (!sig) {
      return new Response("No signature", { status: 400 });
    }

    const body = await req.text();
    
    // Initialize Stripe
    const stripe = new Stripe(Deno.env.get("STRIPE_SECRET_KEY") || "", {
      apiVersion: "2023-10-16",
    });

    // Construct the event - will throw an error if signature verification fails
    let event;
    try {
      event = endpointSecret ? 
        stripe.webhooks.constructEvent(body, sig, endpointSecret) : 
        JSON.parse(body);
    } catch (err) {
      console.error(`Webhook signature verification failed: ${err.message}`);
      return new Response(`Webhook Error: ${err.message}`, { status: 400 });
    }

    console.log(`Webhook event received: ${event.type}`);

    // Initialize Supabase client with service role key for database writes
    const supabaseClient = createClient(
      Deno.env.get("SUPABASE_URL") || "",
      Deno.env.get("SUPABASE_SERVICE_ROLE_KEY") || "",
      { auth: { persistSession: false } }
    );

    // Handle specific events
    switch (event.type) {
      case 'customer.subscription.created':
      case 'customer.subscription.updated': {
        const subscription = event.data.object;
        // Get customer info to find the user_id
        const customer = await stripe.customers.retrieve(subscription.customer);
        const supabaseUserId = customer.metadata?.supabase_uid;
        
        if (!supabaseUserId) {
          console.warn("No Supabase user ID found in customer metadata");
          break;
        }

        // Get the subscription items to determine the plan
        const subscriptionItems = await stripe.subscriptionItems.list({
          subscription: subscription.id,
          expand: ['data.price.product'],
        });
        
        const price = subscriptionItems.data[0].price;
        const product = price.product as Stripe.Product;
        const planName = product.name;
        const isActive = subscription.status === 'active' || subscription.status === 'trialing';
        const currentPeriodEnd = new Date(subscription.current_period_end * 1000).toISOString();
        const renewalDateString = new Date(subscription.current_period_end * 1000).toLocaleDateString();
        const priceString = price.unit_amount ? `$${(price.unit_amount / 100).toFixed(2)}` : 'Custom';
        const userEmail = customer.email;
        const userName = customer.name || customer.email;
        const subscriptionId = subscription.id;

        // Check if cancel_at_period_end changed from false to true
        const wasNotCancelAtPeriodEnd = event.data.previous_attributes?.cancel_at_period_end === false;
        if (subscription.cancel_at_period_end && wasNotCancelAtPeriodEnd) {
          // Send Brevo email for cancel at period end
          try {
            await sendSubscriptionEmail({
              to: userEmail,
              userName,
              planName,
              supportEmail: "support@robometrics.io",
              endDate: renewalDateString,
              templateId: 2 // Cancellation template
            });
            console.log('Cancel-at-period-end email sent via Brevo');
          } catch (err) {
            console.error('Error sending Brevo cancel-at-period-end email:', err);
          }
        }

        // Check if cancel_at_period_end changed from true to false (reactivation)
        const wasCancelAtPeriodEnd = event.data.previous_attributes?.cancel_at_period_end === true;
        if (!subscription.cancel_at_period_end && wasCancelAtPeriodEnd) {
          // Send Brevo reactivation email
          try {
            await sendSubscriptionEmail({
              to: userEmail,
              userName,
              planName,
              price: priceString,
              renewalDate: renewalDateString,
              subscriptionId,
              supportEmail: "support@robometrics.io",
              templateId: BREVO_REACTIVATION_TEMPLATE_ID
            });
            console.log('Reactivation email sent via Brevo');
          } catch (err) {
            console.error('Error sending Brevo reactivation email:', err);
          }
        }

        // Update the subscriptions table
        const { error } = await supabaseClient.from("subscriptions").upsert({
          user_id: supabaseUserId,
          stripe_customer_id: subscription.customer,
          stripe_subscription_id: subscription.id,
          status: isActive ? "active" : "inactive",
          plan_id: price.id,
          plan_name: planName,
          current_period_end: currentPeriodEnd,
          updated_at: new Date().toISOString(),
        }, { onConflict: 'user_id' });

        if (error) {
          console.error('Error updating subscription in database:', error);
        } else {
          console.log('Subscription updated in database:', subscription.id);
        }

        // Send Brevo email for other updates (not cancel at period end)
        try {
          await sendSubscriptionEmail({
            to: userEmail,
            userName,
            planName,
            price: priceString,
            renewalDate: renewalDateString,
            subscriptionId,
            supportEmail: "support@robometrics.io"
          });
          console.log('Subscription email sent via Brevo');
        } catch (err) {
          console.error('Error sending Brevo subscription email:', err);
        }
        break;
      }
      case 'customer.subscription.deleted': {
        const cancelledSubscription = event.data.object;
        const cancelledCustomer = await stripe.customers.retrieve(cancelledSubscription.customer);
        const cancelledUserId = cancelledCustomer.metadata?.supabase_uid;
        const userEmail = cancelledCustomer.email;
        const userName = cancelledCustomer.name || cancelledCustomer.email;
        const planName = "Cancelled";
        const priceString = "";
        const renewalDateString = "";
        const subscriptionId = cancelledSubscription.id;
        
        if (!cancelledUserId) {
          console.warn("No Supabase user ID found in cancelled customer metadata");
          break;
        }
        
        // Update the subscriptions table
        const { error: cancelError } = await supabaseClient.from("subscriptions").upsert({
          user_id: cancelledUserId,
          stripe_customer_id: cancelledSubscription.customer,
          stripe_subscription_id: cancelledSubscription.id,
          status: "inactive",
          updated_at: new Date().toISOString(),
        }, { onConflict: 'user_id' });
        
        if (cancelError) {
          console.error('Error updating cancelled subscription in database:', cancelError);
        } else {
          console.log('Subscription cancelled in database:', cancelledSubscription.id);
        }

        // Send Brevo cancellation email
        try {
          await sendSubscriptionEmail({
            to: userEmail,
            userName,
            planName,
            price: priceString,
            renewalDate: renewalDateString,
            subscriptionId,
            supportEmail: "support@robometrics.io"
          });
          console.log('Cancellation email sent via Brevo');
        } catch (err) {
          console.error('Error sending Brevo cancellation email:', err);
        }
        break;
      }
    }

    return new Response(JSON.stringify({ received: true }), { 
      status: 200,
      headers: { "Content-Type": "application/json" }
    });
  } catch (error) {
    console.error("Error in webhook handler:", error.message);
    return new Response(JSON.stringify({ error: error.message }), { 
      status: 500,
      headers: { "Content-Type": "application/json" }
    });
  }
});
