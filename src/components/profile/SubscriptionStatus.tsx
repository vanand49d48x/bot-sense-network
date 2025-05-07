import React, { useEffect, useState } from "react";
import { Link } from "react-router-dom";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { supabase } from "@/integrations/supabase/client";
import { useToast } from "@/hooks/use-toast";
import { CheckCircle, AlertTriangle, Clock, Shield } from "lucide-react";

interface SubscriptionInfo {
  active: boolean;
  plan: string | null;
  subscription_end: string | null;
  trial_status: string | null;
  days_remaining: number | null;
}

const SubscriptionStatus = () => {
  const [subscription, setSubscription] = useState<SubscriptionInfo | null>(null);
  const [loading, setLoading] = useState(true);
  const [portalLoading, setPortalLoading] = useState(false);
  const { toast } = useToast();

  const checkSubscription = async () => {
    try {
      setLoading(true);
      const { data, error } = await supabase.functions.invoke('check-subscription');
      if (error) throw error;
      setSubscription(data);
    } catch (error: any) {
      console.error("Error checking subscription:", error);
      toast({
        title: "Error",
        description: "Failed to check subscription status",
        variant: "destructive",
      });
    } finally {
      setLoading(false);
    }
  };

  const openCustomerPortal = async () => {
    try {
      setPortalLoading(true);
      const { data, error } = await supabase.functions.invoke('customer-portal');
      if (error) throw error;
      window.location.href = data.url;
    } catch (error: any) {
      console.error("Error opening customer portal:", error);
      toast({
        title: "Error",
        description: "Failed to open subscription management portal",
        variant: "destructive",
      });
    } finally {
      setPortalLoading(false);
    }
  };

  useEffect(() => {
    checkSubscription();
    
    // Refresh subscription status every 10 seconds
    const interval = setInterval(checkSubscription, 10000);
    return () => clearInterval(interval);
  }, []);

  const formatDate = (dateStr: string | null) => {
    if (!dateStr) return "N/A";
    return new Date(dateStr).toLocaleDateString(undefined, {
      year: 'numeric',
      month: 'long',
      day: 'numeric'
    });
  };

  if (loading) {
    return (
      <Card>
        <CardHeader>
          <CardTitle>Subscription</CardTitle>
          <CardDescription>Loading subscription information...</CardDescription>
        </CardHeader>
      </Card>
    );
  }

  // Free Tier and paid plan logic
  const isFreeTier = subscription?.plan === "Free Tier";
  const isTrialActive = isFreeTier && subscription?.trial_status === 'active';
  const isTrialExpired = isFreeTier && subscription?.trial_status === 'expired';
  const isSubscribed = subscription?.active && !isFreeTier;

  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center gap-2">
          {isSubscribed ? (
            <>
              <CheckCircle className="h-5 w-5 text-green-500" />
              <span>Active Subscription</span>
            </>
          ) : isTrialActive ? (
            <>
              <CheckCircle className="h-5 w-5 text-blue-500" />
              <span>Free Tier Active</span>
            </>
          ) : (
            <>
              <AlertTriangle className="h-5 w-5 text-amber-500" />
              <span>{isTrialExpired ? "Free Tier Expired" : "No Active Subscription"}</span>
            </>
          )}
        </CardTitle>
        <CardDescription>
          DEBUG-VISHAL: {isSubscribed ? "PAID" : isTrialActive ? "FREE-TIER-ACTIVE" : isTrialExpired ? "FREE-TIER-EXPIRED" : "NONE"} | {subscription?.plan}
          {isSubscribed
            ? `You are currently on the ${subscription?.plan} plan`
            : isTrialActive
              ? `Your Free Tier expires in ${subscription?.days_remaining} days`
              : isTrialExpired
                ? "Your Free Tier has expired. Upgrade to continue using premium features."
                : "Start your free trial to access premium features"}
        </CardDescription>
      </CardHeader>
      
      <CardContent>
        <div className="space-y-4">
          <div className="flex items-center gap-2">
            <Clock className="h-4 w-4 text-muted-foreground" />
            <span className="text-sm">
              {isFreeTier
                ? (subscription?.trial_status === "active"
                    ? `Expires on ${formatDate(subscription?.subscription_end ?? null)}`
                    : `Expired on ${formatDate(subscription?.subscription_end ?? null)}`)
                : subscription?.subscription_end
                  ? `Renews on ${formatDate(subscription.subscription_end)}`
                  : ''}
            </span>
          </div>
          
          <div className="flex items-center gap-2">
            <Shield className="h-4 w-4 text-muted-foreground" />
            <span className="text-sm">
              Manage your subscription details securely via Stripe
            </span>
          </div>
        </div>

        {!subscription?.active && (
          <div className="p-4 bg-muted/50 rounded-md">
            <p className="text-sm">
              Upgrade to a paid plan to access premium features like:
            </p>
            <ul className="mt-2 space-y-1 text-sm">
              <li className="flex items-center gap-1">
                <CheckCircle className="h-3 w-3 text-green-500" />
                <span>Monitor more robots</span>
              </li>
              <li className="flex items-center gap-1">
                <CheckCircle className="h-3 w-3 text-green-500" />
                <span>Extended telemetry history</span>
              </li>
              <li className="flex items-center gap-1">
                <CheckCircle className="h-3 w-3 text-green-500" />
                <span>Advanced alerts and notifications</span>
              </li>
            </ul>
          </div>
        )}
      </CardContent>
      
      <CardFooter className="flex flex-col gap-2">
        {isSubscribed ? (
          <Button 
            variant="outline" 
            className="w-full" 
            onClick={openCustomerPortal}
            disabled={portalLoading}
          >
            {portalLoading ? "Opening Portal..." : "Manage Subscription"}
          </Button>
        ) : (
          <Button 
            className="w-full" 
            asChild
            variant={isTrialActive ? "outline" : "default"}>
            <Link to="/pricing">
              {isTrialActive ? "Upgrade Now" : isTrialExpired ? "Upgrade to Continue" : "Start Free Tier"}
            </Link>
          </Button>
        )}
        
        <Button 
          variant="ghost" 
          className="w-full text-sm" 
          onClick={checkSubscription}
          disabled={loading}
        >
          Refresh Status
        </Button>
      </CardFooter>
    </Card>
  );
};

export default SubscriptionStatus;
