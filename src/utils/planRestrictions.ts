
import { useQuery } from "@tanstack/react-query";
import { supabase } from "@/integrations/supabase/client";

// Define subscription tier limits as an interface
export interface PlanLimits {
  robotLimit: number;
  telemetryDays: number;
  customTelemetryTypes: number;
  alertsPerDay: number;
  supportLevel: "basic" | "priority" | "dedicated";
  apiAccess: boolean;
  advancedAnalytics: boolean;
}

// Default limits for each plan - used as fallback if database fetch fails
export const DEFAULT_PLAN_LIMITS: Record<string, PlanLimits> = {
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
export const DEFAULT_LIMITS: PlanLimits = DEFAULT_PLAN_LIMITS["Free Tier"];

// Hook to get the current user's subscription plan and limits
export function useSubscriptionLimits() {
  const { data, isLoading, error, refetch } = useQuery({
    queryKey: ['subscription-check'],
    queryFn: async () => {
      try {
        console.log("Fetching subscription data from edge function...");
        // Call the check-subscription edge function to get subscription data
        const { data, error } = await supabase.functions.invoke('check-subscription');
        
        if (error) {
          console.error("Error from check-subscription function:", error);
          throw error;
        }
        console.log("Subscription data from edge function:", data);
        return data;
      } catch (err) {
        console.error("Error checking subscription:", err);
        return { active: false, plan: "Free Tier" };
      }
    },
    refetchInterval: 60000 * 2, // Refresh every 2 minutes instead of 5 to pick up changes faster
    staleTime: 30000, // Consider data stale after 30 seconds
  });

  // Get plan limits from database or fall back to defaults
  const { data: planLimitsData, isLoading: planLimitsLoading } = useQuery({
    queryKey: ['plan-limits'],
    queryFn: async () => {
      try {
        console.log("Fetching plan limits from database...");
        // Fetch plan limits from the database
        const { data, error } = await supabase
          .from('plan_limits')
          .select('*');
          
        if (error) {
          console.error("Error fetching plan limits:", error);
          throw error;
        }
        
        console.log("Plan limits from database:", data);
        
        // Convert database results to our format
        const limitsMap: Record<string, PlanLimits> = {};
        
        if (data && data.length > 0) {
          // Transform database records into our PlanLimits format
          data.forEach(item => {
            // For Enterprise plan, null values mean unlimited
            const robotLimit = item.robot_limit === null ? Infinity : item.robot_limit;
            const telemetryDays = item.telemetry_days === null ? Infinity : item.telemetry_days;
            const customTelemetryTypes = item.custom_telemetry_types === null ? Infinity : item.custom_telemetry_types;
            const alertsPerDay = item.alerts_per_day === null ? Infinity : item.alerts_per_day;
            
            limitsMap[item.plan_name] = {
              robotLimit,
              telemetryDays,
              customTelemetryTypes,
              alertsPerDay,
              supportLevel: item.support_level || DEFAULT_PLAN_LIMITS["Free Tier"]?.supportLevel,
              apiAccess: item.api_access === undefined ? DEFAULT_PLAN_LIMITS["Free Tier"]?.apiAccess : item.api_access,
              advancedAnalytics: item.advanced_analytics === undefined ? DEFAULT_PLAN_LIMITS["Free Tier"]?.advancedAnalytics : item.advanced_analytics,
            };
          });
          return limitsMap;
        }
        
        // If no data, use defaults
        return DEFAULT_PLAN_LIMITS;
      } catch (err) {
        console.error("Error fetching plan limits:", err);
        return DEFAULT_PLAN_LIMITS;
      }
    },
    staleTime: 300000, // Consider data fresh for 5 minutes
  });

  // Get the current plan name from subscription check
  let planName = (data?.plan || "Free Tier");
  
  // Normalize the plan name case
  if (planName && typeof planName === 'string') {
    // Handle special case for exact match "STARTER"
    if (planName === "STARTER") {
      planName = "Starter";
    }
    // For other cases, try to normalize
    else if (!planLimitsData?.[planName]) {
      // Try to find a case-insensitive match
      const planKeys = Object.keys(planLimitsData || {});
      const matchedPlan = planKeys.find(key => key.toLowerCase() === planName.toLowerCase());
      if (matchedPlan) {
        planName = matchedPlan;
      }
    }
  }
  
  // Log the plan name for debugging purposes
  console.log("Current plan name:", planName, "Original from API:", data?.plan);
  console.log("Available plans in limits:", Object.keys(planLimitsData || {}));
  
  // Use database limits if available, otherwise use defaults
  const planLimits = planLimitsData || DEFAULT_PLAN_LIMITS;
  
  // Get limits for the current plan, or fallback to default limits
  const limits = planLimits[planName] || DEFAULT_LIMITS;
  
  // Log the limits for debugging
  console.log("Applied limits for plan:", planName, limits);
  
  // Add additional derived data
  const remainingDays = data?.days_remaining || 0;
  const isActive = data?.active || false;
  const isTrialActive = planName === "Free Tier" && data?.trial_status === 'active';
  const isTrialExpired = planName === "Free Tier" && data?.trial_status === 'expired';

  return {
    planName,
    limits,
    isActive,
    isLoading: isLoading || planLimitsLoading,
    isTrialActive,
    isTrialExpired,
    remainingDays,
    error,
    refetch // Expose refetch function to manually refresh subscription data
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
