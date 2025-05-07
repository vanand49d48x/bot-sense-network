
import { useQuery } from "@tanstack/react-query";
import { supabase } from "@/integrations/supabase/client";

// Define subscription tier limits
export interface PlanLimits {
  robotLimit: number;
  telemetryDays: number;
  customTelemetryTypes: number;
  alertsPerDay: number;
  supportLevel: "basic" | "priority" | "dedicated";
  apiAccess: boolean;
  advancedAnalytics: boolean;
}

// Mapping of subscription plans to their limits
export const PLAN_LIMITS: Record<string, PlanLimits> = {
  "Free Tier": {
    robotLimit: 2,
    telemetryDays: 7,
    customTelemetryTypes: 3,
    alertsPerDay: 10,
    supportLevel: "basic",
    apiAccess: false,
    advancedAnalytics: false
  },
  "Starter": {
    robotLimit: 5,
    telemetryDays: 30,
    customTelemetryTypes: 10,
    alertsPerDay: 50,
    supportLevel: "basic",
    apiAccess: true,
    advancedAnalytics: false
  },
  "Growth": {
    robotLimit: 20,
    telemetryDays: 90,
    customTelemetryTypes: 25,
    alertsPerDay: 200,
    supportLevel: "priority",
    apiAccess: true,
    advancedAnalytics: true
  },
  "Pro": {
    robotLimit: 100,
    telemetryDays: 365,
    customTelemetryTypes: 50,
    alertsPerDay: 1000,
    supportLevel: "dedicated",
    apiAccess: true,
    advancedAnalytics: true
  },
  "Enterprise": {
    robotLimit: Infinity,
    telemetryDays: Infinity,
    customTelemetryTypes: Infinity,
    alertsPerDay: Infinity,
    supportLevel: "dedicated",
    apiAccess: true,
    advancedAnalytics: true
  }
};

// Default limits for users with no subscription data
export const DEFAULT_LIMITS: PlanLimits = PLAN_LIMITS["Free Tier"];

// Hook to get the current user's subscription plan and limits
export function useSubscriptionLimits() {
  const { data, isLoading, error } = useQuery({
    queryKey: ['subscription-check'],
    queryFn: async () => {
      try {
        const { data, error } = await supabase.functions.invoke('check-subscription');
        
        if (error) throw error;
        return data;
      } catch (err) {
        console.error("Error checking subscription:", err);
        return { active: false, plan: "Free Tier" };
      }
    },
    refetchInterval: 60000 * 5, // Refresh every 5 minutes
  });

  // Get the limits for the current plan
  const planName = data?.plan || "Free Tier";
  const limits = PLAN_LIMITS[planName] || DEFAULT_LIMITS;
  
  // Add additional derived data
  const remainingDays = data?.days_remaining || 0;
  const isActive = data?.active || false;
  const isTrialActive = planName === "Free Tier" && data?.trial_status === 'active';
  const isTrialExpired = planName === "Free Tier" && data?.trial_status === 'expired';

  return {
    planName,
    limits,
    isActive,
    isLoading,
    isTrialActive,
    isTrialExpired,
    remainingDays,
    error
  };
}

// Utility function to check if a limit would be exceeded
export function wouldExceedLimit(
  currentCount: number, 
  action: 'add' | 'remove' = 'add', 
  planLimit: number
): boolean {
  if (planLimit === Infinity) return false;
  
  if (action === 'add') {
    return currentCount + 1 > planLimit;
  }
  
  return false;
}
