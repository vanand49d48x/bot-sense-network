
// This file contains helper functions for Stripe checkout integration
// Note: This is just a template - actual implementation requires Supabase edge functions

import { supabase } from "@/integrations/supabase/client";

export const createCheckoutSession = async (priceId: string) => {
  try {
    // Call the Supabase edge function that creates a Stripe checkout session
    const { data, error } = await supabase.functions.invoke('create-checkout', {
      body: { priceId }
    });

    if (error) throw new Error(error.message);
    if (!data || !data.url) throw new Error('No checkout URL returned');

    return data.url;
  } catch (error) {
    console.error('Error creating checkout session:', error);
    throw error;
  }
};

export const checkSubscriptionStatus = async () => {
  try {
    // Call the Supabase edge function that checks subscription status
    const { data, error } = await supabase.functions.invoke('check-subscription');

    if (error) throw new Error(error.message);
    return data;
  } catch (error) {
    console.error('Error checking subscription status:', error);
    throw error;
  }
};

export const openCustomerPortal = async () => {
  try {
    // Call the Supabase edge function that creates a Stripe customer portal session
    const { data, error } = await supabase.functions.invoke('customer-portal');

    if (error) throw new Error(error.message);
    if (!data || !data.url) throw new Error('No portal URL returned');

    return data.url;
  } catch (error) {
    console.error('Error opening customer portal:', error);
    throw error;
  }
};
