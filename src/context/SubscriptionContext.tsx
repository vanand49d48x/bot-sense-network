
import React, { createContext, useContext, useState, useEffect } from "react";
import { supabase } from "@/integrations/supabase/client";
import { useAuth } from "@/context/AuthContext";
import { toast } from "@/components/ui/sonner";

export type SubscriptionTier = "free" | "starter" | "growth" | "pro" | "enterprise";

export interface Subscription {
  id: string;
  planId: string;
  planName: string;
  status: string;
  currentPeriodEnd: string | null;
}

interface SubscriptionContextType {
  subscription: Subscription | null;
  isLoading: boolean;
  refetchSubscription: () => Promise<void>;
  subscriptionTier: SubscriptionTier;
  isSubscriptionActive: boolean;
  openCustomerPortal: () => Promise<void>;
}

const SubscriptionContext = createContext<SubscriptionContextType | undefined>(undefined);

export const useSubscription = () => {
  const context = useContext(SubscriptionContext);
  if (context === undefined) {
    throw new Error("useSubscription must be used within a SubscriptionProvider");
  }
  return context;
};

export const SubscriptionProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const { user } = useAuth();
  const [subscription, setSubscription] = useState<Subscription | null>(null);
  const [isLoading, setIsLoading] = useState(false);

  const fetchSubscription = async () => {
    if (!user) {
      setSubscription(null);
      return;
    }

    try {
      setIsLoading(true);
      const { data, error } = await supabase.functions.invoke("check-subscription");
      
      if (error) {
        throw error;
      }
      
      if (data.subscription) {
        setSubscription({
          id: data.subscription.id,
          planId: data.subscription.plan_id || "free",
          planName: data.subscription.plan_name || "Free",
          status: data.subscription.status,
          currentPeriodEnd: data.subscription.current_period_end,
        });
      } else {
        setSubscription(null);
      }
    } catch (error) {
      console.error("Error fetching subscription:", error);
      toast.error("Failed to load subscription information");
    } finally {
      setIsLoading(false);
    }
  };

  useEffect(() => {
    fetchSubscription();
  }, [user]);

  const openCustomerPortal = async () => {
    if (!user) {
      toast.error("You need to be logged in to manage your subscription");
      return;
    }

    try {
      setIsLoading(true);
      const { data, error } = await supabase.functions.invoke("customer-portal");
      
      if (error) {
        throw error;
      }
      
      if (data.url) {
        window.location.href = data.url;
      }
    } catch (error) {
      console.error("Error opening customer portal:", error);
      toast.error("Failed to open subscription management");
    } finally {
      setIsLoading(false);
    }
  };

  // Determine the current subscription tier
  const subscriptionTier: SubscriptionTier = 
    !subscription || subscription.status !== "active" 
      ? "free" 
      : (subscription.planId as SubscriptionTier);

  // Check if subscription is active
  const isSubscriptionActive = subscription?.status === "active";

  return (
    <SubscriptionContext.Provider
      value={{
        subscription,
        isLoading,
        refetchSubscription: fetchSubscription,
        subscriptionTier,
        isSubscriptionActive,
        openCustomerPortal,
      }}
    >
      {children}
    </SubscriptionContext.Provider>
  );
};
