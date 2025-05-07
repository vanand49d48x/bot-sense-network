import React, { useEffect, useState } from "react";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { useToast } from "@/hooks/use-toast";
import { supabase } from "@/integrations/supabase/client";
import { Loader2, CreditCard, AlertCircle, Check } from "lucide-react";
import { Alert, AlertDescription, AlertTitle } from "@/components/ui/alert";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog";
import { useAuth } from "@/context/AuthContext";
import { Link } from "react-router-dom";

interface Subscription {
  active: boolean;
  plan: string | null;
  subscription_end: string | null;
  subscription_id?: string;
  cancel_at_period_end?: boolean;
  trial_status?: string;
  days_remaining?: number;
  price?: {
    amount: number;
    currency: string;
  };
}

interface Plan {
  id: string;
  name: string;
  price: string;
  description: string;
  features: string[];
  bestFor: string;
  highlight?: boolean;
  buttonText: string;
  robotLimit: number;
  telemetryDays: number;
  priceId?: string;
}

const plans: Plan[] = [
  {
    id: "free",
    name: "Free",
    price: "$0",
    description: "Basic monitoring for hobby projects",
    bestFor: "Hobbyists & Students",
    features: [
      "Monitor up to 2 robots",
      "Basic telemetry",
      "7 days of history",
      "Email alerts"
    ],
    buttonText: "Get Started",
    robotLimit: 2,
    telemetryDays: 7
  },
  {
    id: "starter",
    name: "Starter",
    price: "$29",
    description: "For small teams with growing needs",
    bestFor: "Small Teams",
    features: [
      "Monitor up to 5 robots",
      "Advanced telemetry",
      "30 days of history",
      "SMS & Email alerts",
      "API access"
    ],
    highlight: true,
    buttonText: "Subscribe",
    robotLimit: 5,
    telemetryDays: 30,
    priceId: "STRIPE_STARTER_PRICE_ID"
  },
  {
    id: "growth",
    name: "Growth",
    price: "$79",
    description: "For teams managing multiple robots",
    bestFor: "Medium Businesses",
    features: [
      "Monitor up to 20 robots",
      "Premium telemetry",
      "90 days of history",
      "Custom alerts",
      "Priority support",
      "Advanced analytics"
    ],
    buttonText: "Subscribe",
    robotLimit: 20,
    telemetryDays: 90,
    priceId: "STRIPE_GROWTH_PRICE_ID"
  },
  {
    id: "pro",
    name: "Pro",
    price: "$199",
    description: "For large deployments & organizations",
    bestFor: "Large Organizations",
    features: [
      "Monitor up to 100 robots",
      "Enterprise telemetry",
      "1 year of history",
      "Custom integrations",
      "24/7 support",
      "Advanced security",
      "Custom reporting"
    ],
    buttonText: "Subscribe",
    robotLimit: 100,
    telemetryDays: 365,
    priceId: "STRIPE_PRO_PRICE_ID"
  },
  {
    id: "enterprise",
    name: "Enterprise",
    price: "Custom",
    description: "Custom solutions for large organizations",
    bestFor: "Fortune 500",
    features: [
      "Unlimited robots",
      "Enterprise telemetry",
      "Unlimited history",
      "Custom integrations",
      "Dedicated support",
      "On-premise option",
      "SLA guarantees",
      "Custom development"
    ],
    buttonText: "Contact Sales",
    robotLimit: 9999,
    telemetryDays: 9999
  }
];

export function SubscriptionManager() {
  const [subscription, setSubscription] = useState<Subscription | null>(null);
  const [loading, setLoading] = useState(true);
  const [portalLoading, setPortalLoading] = useState(false);
  const [isPlansOpen, setIsPlansOpen] = useState(false);
  const { toast } = useToast();
  const { user } = useAuth();

  useEffect(() => {
    fetchSubscription();
  }, []);

  const fetchSubscription = async () => {
    try {
      console.log("Fetching subscription data...");
      setLoading(true);
      const { data, error } = await supabase.functions.invoke('check-subscription');
      
      console.log("Subscription response:", { data, error });
      
      if (error) throw error;
      
      setSubscription(data);
      console.log("Subscription set:", data);
    } catch (error: any) {
      console.error('Error fetching subscription:', error);
      toast({
        title: "Error",
        description: "Failed to load subscription details",
        variant: "destructive",
      });
    } finally {
      setLoading(false);
    }
  };

  const handleManageSubscription = async () => {
    try {
      console.log("Opening customer portal...");
      setPortalLoading(true);
      const { data, error } = await supabase.functions.invoke('customer-portal');
      
      console.log("Customer portal response:", { data, error });
      
      if (error) throw error;
      
      // Redirect to Stripe Customer Portal
      window.location.href = data.url;
    } catch (error: any) {
      console.error('Error opening customer portal:', error);
      toast({
        title: "Error",
        description: "Failed to open subscription management portal",
        variant: "destructive",
      });
    } finally {
      setPortalLoading(false);
    }
  };

  const handleSubscription = async (plan: Plan) => {
    if (plan.id === "free") {
      // Navigate to signup for free plan
      window.location.href = "/auth?tab=signup";
      return;
    }

    try {
      if (!user) {
        // Not logged in - redirect to auth page with return URL to pricing and the priceId
        window.location.href = `/auth?returnUrl=/pricing&priceId=${plan.priceId}`;
        return;
      }

      // For paid plans, redirect to Stripe checkout
      if (plan.priceId) {
        toast({
          title: "Redirecting to checkout",
          description: "Please wait while we prepare your checkout session",
        });

        const { data, error } = await supabase.functions.invoke('create-checkout', {
          body: { priceId: plan.priceId }
        });

        if (error) {
          throw error;
        }

        // Redirect to Stripe Checkout
        window.location.href = data.url;
      }
    } catch (error: any) {
      console.error("Error creating checkout session:", error);
      toast({
        title: "Error",
        description: error.message || "Failed to create checkout session",
        variant: "destructive",
      });
    }
  };

  console.log("Current subscription state:", { subscription, loading, portalLoading });

  // Free Tier and paid plan logic
  const isFreeTier = subscription?.plan === "Free Tier";
  const isTrialActive = isFreeTier && subscription?.trial_status === 'active';
  const isTrialExpired = isFreeTier && subscription?.trial_status === 'expired';
  const isSubscribed = subscription?.active && !isFreeTier;

  const formatDate = (dateStr: string | null) => {
    if (!dateStr) return "N/A";
    return new Date(dateStr).toLocaleDateString(undefined, {
      year: 'numeric',
      month: 'long',
      day: 'numeric'
    });
  };

  console.log('DEBUG: plan:', subscription?.plan, 'trial_status:', subscription?.trial_status, 'active:', subscription?.active);

  if (loading) {
    return (
      <Card>
        <CardHeader>
          <CardTitle>Subscription</CardTitle>
          <CardDescription>Loading subscription details...</CardDescription>
        </CardHeader>
        <CardContent className="flex justify-center py-6">
          <Loader2 className="h-6 w-6 animate-spin" />
        </CardContent>
      </Card>
    );
  }

  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center gap-2">
          {isSubscribed ? (
            <>
              <Check className="h-5 w-5 text-green-500" />
              <span>Active Subscription</span>
            </>
          ) : isTrialActive ? (
            <>
              <Check className="h-5 w-5 text-blue-500" />
              <span>Free Tier Active</span>
            </>
          ) : (
            <>
              <AlertCircle className="h-5 w-5 text-amber-500" />
              <span>{isTrialExpired ? "Free Tier Expired" : "No Active Subscription"}</span>
            </>
          )}
        </CardTitle>
        <CardDescription>
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
            <CreditCard className="h-4 w-4 text-muted-foreground" />
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

          {isSubscribed && subscription?.plan && (
            <div className="bg-muted/50 p-4 rounded-md space-y-2">
              <div className="flex justify-between items-center">
                <span className="text-sm font-medium">Current Plan</span>
                <span className="text-sm font-semibold">{subscription.plan}</span>
              </div>
              {subscription.price && (
                <div className="flex justify-between items-center">
                  <span className="text-sm font-medium">Monthly Price</span>
                  <span className="text-sm font-semibold">
                    ${(subscription.price.amount / 100).toFixed(2)} {subscription.price.currency.toUpperCase()}
                  </span>
                </div>
              )}
              <div className="flex justify-between items-center">
                <span className="text-sm font-medium">Billing Cycle</span>
                <span className="text-sm">Monthly</span>
              </div>
              {subscription.cancel_at_period_end && (
                <div className="mt-2 p-2 bg-amber-100 dark:bg-amber-900/20 rounded text-amber-800 dark:text-amber-200 text-sm">
                  Your subscription will end on {formatDate(subscription.subscription_end)}
                </div>
              )}
            </div>
          )}
        </div>
      </CardContent>
      <CardFooter>
        {isSubscribed ? (
          <Button onClick={handleManageSubscription} className="w-full" disabled={portalLoading}>
            {portalLoading ? "Opening Portal..." : "Manage Subscription"}
          </Button>
        ) : (
          isTrialActive || isTrialExpired ? (
            <Button asChild className="w-full" variant={isTrialActive ? "outline" : "default"}>
              <Link to="/pricing">
                {isTrialActive ? "Upgrade Now" : "Upgrade to Continue"}
              </Link>
            </Button>
          ) : null
        )}
      </CardFooter>
    </Card>
  );
} 