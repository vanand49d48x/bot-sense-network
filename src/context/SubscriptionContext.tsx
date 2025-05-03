
import React, { createContext, useState, useEffect, useContext } from "react";
import { useAuth } from "@/context/AuthContext";
import { supabase } from "@/integrations/supabase/client";
import { useToast } from "@/components/ui/use-toast";
import { useLocation } from "react-router-dom";

type SubscriptionPlan = "Starter" | "Growth" | "Pro" | "Custom" | null;

type SubscriptionContextType = {
  isSubscribed: boolean;
  plan: SubscriptionPlan;
  currentPeriodEnd: Date | null;
  isLoading: boolean;
  checkSubscription: () => Promise<void>;
  openCustomerPortal: () => Promise<void>;
};

const SubscriptionContext = createContext<SubscriptionContextType | undefined>(undefined);

export function SubscriptionProvider({ children }: { children: React.ReactNode }) {
  const [isSubscribed, setIsSubscribed] = useState<boolean>(false);
  const [plan, setPlan] = useState<SubscriptionPlan>(null);
  const [currentPeriodEnd, setCurrentPeriodEnd] = useState<Date | null>(null);
  const [isLoading, setIsLoading] = useState<boolean>(true);
  const { user, session } = useAuth();
  const { toast } = useToast();
  const location = useLocation();

  // Check for payment=success in URL params
  useEffect(() => {
    const params = new URLSearchParams(location.search);
    const paymentStatus = params.get('payment');
    
    if (paymentStatus === 'success') {
      toast({
        title: "Payment successful!",
        description: "Your subscription has been activated.",
      });
      // Remove the query param from the URL without refreshing the page
      const newUrl = window.location.pathname;
      window.history.replaceState({}, document.title, newUrl);
      
      // Delay checking subscription to allow Stripe to process the payment
      setTimeout(() => {
        checkSubscription();
      }, 2000);
    } else if (paymentStatus === 'canceled') {
      toast({
        title: "Payment canceled",
        description: "Your subscription was not processed.",
        variant: "destructive",
      });
      // Remove the query param from the URL without refreshing the page
      const newUrl = window.location.pathname;
      window.history.replaceState({}, document.title, newUrl);
    }
  }, [location]);

  const checkSubscription = async () => {
    if (!user || !session?.access_token) {
      setIsLoading(false);
      return;
    }

    try {
      setIsLoading(true);
      const { data, error } = await supabase.functions.invoke("check-subscription");
      
      if (error) {
        throw error;
      }
      
      setIsSubscribed(data.subscribed);
      setPlan(data.plan);
      setCurrentPeriodEnd(data.current_period_end ? new Date(data.current_period_end) : null);
    } catch (error) {
      console.error("Error checking subscription:", error);
      toast({
        title: "Error checking subscription",
        description: "Unable to verify your subscription status.",
        variant: "destructive",
      });
    } finally {
      setIsLoading(false);
    }
  };

  const openCustomerPortal = async () => {
    if (!user) {
      toast({
        title: "Not logged in",
        description: "Please log in to manage your subscription.",
        variant: "destructive",
      });
      return;
    }

    try {
      const { data, error } = await supabase.functions.invoke("customer-portal");
      
      if (error) {
        throw error;
      }
      
      if (data?.url) {
        window.location.href = data.url;
      } else {
        throw new Error("No portal URL returned");
      }
    } catch (error) {
      console.error("Error opening customer portal:", error);
      toast({
        title: "Error",
        description: "Unable to open subscription management portal.",
        variant: "destructive",
      });
    }
  };

  // Check subscription when user changes or on mount
  useEffect(() => {
    if (user) {
      checkSubscription();
    } else {
      setIsLoading(false);
      setIsSubscribed(false);
      setPlan(null);
      setCurrentPeriodEnd(null);
    }
  }, [user]);

  return (
    <SubscriptionContext.Provider
      value={{
        isSubscribed,
        plan,
        currentPeriodEnd,
        isLoading,
        checkSubscription,
        openCustomerPortal
      }}
    >
      {children}
    </SubscriptionContext.Provider>
  );
}

export function useSubscription() {
  const context = useContext(SubscriptionContext);
  if (context === undefined) {
    throw new Error("useSubscription must be used within a SubscriptionProvider");
  }
  return context;
}
