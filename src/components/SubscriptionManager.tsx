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

interface Subscription {
  active: boolean;
  plan: string | null;
  subscription_end: string | null;
  subscription_id?: string;
  cancel_at_period_end?: boolean;
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

  if (!subscription || !subscription.active) {
    return (
      <Card>
        <CardHeader>
          <CardTitle>Subscription</CardTitle>
          <CardDescription>
            {subscription?.cancel_at_period_end 
              ? "Your subscription will end on " + new Date(subscription.subscription_end!).toLocaleDateString()
              : "You don't have an active subscription"}
          </CardDescription>
        </CardHeader>
        <CardContent>
          <Alert>
            <AlertCircle className="h-4 w-4" />
            <AlertTitle>
              {subscription?.cancel_at_period_end ? "Subscription Ending" : "No Active Plan"}
            </AlertTitle>
            <AlertDescription>
              {subscription?.cancel_at_period_end
                ? "Your subscription will remain active until the end of the current billing period."
                : "Subscribe to a plan to unlock premium features and increase your robot limits."}
            </AlertDescription>
          </Alert>
        </CardContent>
        <CardFooter>
          <Dialog open={isPlansOpen} onOpenChange={setIsPlansOpen}>
            <DialogTrigger asChild>
              <Button>View Plans</Button>
            </DialogTrigger>
            <DialogContent className="max-w-6xl">
              <DialogHeader>
                <DialogTitle>Choose Your Plan</DialogTitle>
                <DialogDescription>
                  Select the plan that best fits your needs
                </DialogDescription>
              </DialogHeader>
              <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-5 gap-4 mt-4">
                {plans.map((plan) => (
                  <Card 
                    key={plan.id}
                    className={`flex flex-col transition-all duration-200 ${plan.highlight ? 'border-primary shadow-lg' : ''} hover:border-primary hover:shadow-xl`}
                  >
                    <CardHeader>
                      <div className="flex items-center justify-between">
                        <CardTitle>{plan.name}</CardTitle>
                        {plan.highlight && (
                          <span className="bg-primary/10 text-primary text-xs px-2 py-1 rounded font-medium">
                            Popular
                          </span>
                        )}
                      </div>
                      <div className="mt-2">
                        <span className="text-3xl font-bold">{plan.price}</span>
                        {plan.id !== "free" && <span className="text-muted-foreground">/month</span>}
                      </div>
                      <CardDescription className="mt-2">{plan.description}</CardDescription>
                    </CardHeader>
                    <CardContent className="flex-grow">
                      <div className="bg-muted/50 text-sm py-1 px-3 rounded mb-4">
                        Best for: {plan.bestFor}
                      </div>
                      <ul className="space-y-2 text-sm">
                        {plan.features.map((feature, i) => (
                          <li key={i} className="flex items-center gap-2">
                            <Check size={16} className="text-primary shrink-0" />
                            <span>{feature}</span>
                          </li>
                        ))}
                      </ul>
                    </CardContent>
                    <CardFooter>
                      <Button 
                        className="w-full" 
                        variant={plan.id === "free" ? "outline" : "default"}
                        onClick={() => handleSubscription(plan)}
                      >
                        {plan.buttonText}
                      </Button>
                    </CardFooter>
                  </Card>
                ))}
              </div>
            </DialogContent>
          </Dialog>
        </CardFooter>
      </Card>
    );
  }

  const renewalDate = subscription.subscription_end 
    ? new Date(subscription.subscription_end).toLocaleDateString()
    : 'Unknown';

  return (
    <Card>
      <CardHeader>
        <CardTitle>Subscription</CardTitle>
        <CardDescription>Your current subscription plan</CardDescription>
      </CardHeader>
      <CardContent className="space-y-4">
        <div className="flex items-center justify-between">
          <div>
            <h3 className="font-medium">{subscription.plan}</h3>
            <p className="text-sm text-muted-foreground">Active Plan</p>
          </div>
          <div className="text-right">
            <p className="text-sm font-medium">
              {subscription.cancel_at_period_end 
                ? `Ends on ${renewalDate}`
                : `Renews on ${renewalDate}`}
            </p>
          </div>
        </div>
      </CardContent>
      <CardFooter>
        <Button 
          className="w-full" 
          onClick={handleManageSubscription}
          disabled={portalLoading}
        >
          {portalLoading ? (
            <>
              <Loader2 className="mr-2 h-4 w-4 animate-spin" />
              Opening Portal...
            </>
          ) : (
            <>
              <CreditCard className="mr-2 h-4 w-4" />
              Manage Subscription
            </>
          )}
        </Button>
      </CardFooter>
    </Card>
  );
} 