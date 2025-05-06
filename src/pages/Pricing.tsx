
import React, { useState } from "react";
import { MainLayout } from "@/components/layout/MainLayout";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Check, AlertCircle, Loader2 } from "lucide-react";
import { useNavigate, useSearchParams } from "react-router-dom";
import { supabase } from "@/integrations/supabase/client";
import { useAuth } from "@/context/AuthContext";
import { useSubscription } from "@/context/SubscriptionContext";
import { toast } from "@/components/ui/sonner";
import { Alert, AlertDescription, AlertTitle } from "@/components/ui/alert";

const plans = [
  {
    id: "free",
    name: "Free",
    price: "$0/month",
    bestFor: "Hobbyists, solo developers",
    features: [
      "1 robot",
      "3 days telemetry history",
      "Live dashboard",
      "Basic API access"
    ],
  },
  {
    id: "starter",
    name: "Starter",
    price: "$19/month",
    bestFor: "Startup teams, testers",
    features: [
      "Up to 5 robots",
      "7 days history",
      "Email alerts",
      "ROS/Arduino SDK support"
    ],
    popular: true,
  },
  {
    id: "growth",
    name: "Growth",
    price: "$49/month",
    bestFor: "Growing teams, labs",
    features: [
      "25 robots",
      "30-day telemetry retention",
      "Custom metrics",
      "Real-time alerts",
      "Location map"
    ],
  },
  {
    id: "pro",
    name: "Pro",
    price: "$149/month",
    bestFor: "Industrial use, fleets",
    features: [
      "100+ robots",
      "90-day history",
      "SLA + Priority support",
      "Custom alerting rules",
      "Export + team access"
    ],
  },
  {
    id: "enterprise",
    name: "Enterprise",
    price: "Custom",
    bestFor: "OEMs, manufacturers",
    features: [
      "Unlimited robots",
      "Custom data retention",
      "SSO / Audit logs",
      "Dedicated onboarding",
      "On-premise / MQTT support"
    ],
    custom: true,
  }
];

const Pricing = () => {
  const [searchParams] = useSearchParams();
  const navigate = useNavigate();
  const { user } = useAuth();
  const { subscriptionTier, isSubscriptionActive } = useSubscription();
  const [loadingPlanId, setLoadingPlanId] = useState<string | null>(null);
  
  const canceled = searchParams.get("canceled") === "true";
  
  const handleSelectPlan = async (planId: string) => {
    // Free plan doesn't require checkout
    if (planId === "free") {
      return;
    }
    
    // Enterprise plan requires contact
    if (planId === "enterprise") {
      navigate("/contact");
      return;
    }
    
    try {
      setLoadingPlanId(planId);
      
      if (!user) {
        // Store the selected plan in sessionStorage and redirect to auth
        sessionStorage.setItem("selectedPlan", planId);
        navigate("/auth", { state: { redirectTo: "/pricing" } });
        return;
      }
      
      // Create a checkout session
      const { data, error } = await supabase.functions.invoke("create-checkout", {
        body: {
          planId,
          userId: user.id
        }
      });
      
      if (error) {
        throw error;
      }
      
      // Redirect to Stripe Checkout
      if (data.url) {
        window.location.href = data.url;
      } else {
        throw new Error("No checkout URL returned");
      }
    } catch (error) {
      console.error("Error creating checkout session:", error);
      toast.error("Failed to start checkout process");
    } finally {
      setLoadingPlanId(null);
    }
  };

  return (
    <MainLayout>
      <div className="container py-10 max-w-7xl mx-auto">
        <div className="text-center mb-10">
          <h1 className="text-4xl font-bold mb-4">Choose the Right Plan for Your Needs</h1>
          <p className="text-lg text-muted-foreground max-w-3xl mx-auto">
            From individual robots to large fleets, we have a plan for every scale of operation.
            All plans include our core monitoring features.
          </p>
        </div>
        
        {canceled && (
          <Alert variant="destructive" className="mb-8 max-w-3xl mx-auto">
            <AlertCircle className="h-4 w-4" />
            <AlertTitle>Checkout canceled</AlertTitle>
            <AlertDescription>
              Your checkout process was canceled. No charges were made. You can try again whenever you're ready.
            </AlertDescription>
          </Alert>
        )}
        
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 xl:grid-cols-5 gap-6">
          {plans.map((plan) => {
            const isCurrentPlan = subscriptionTier === plan.id && isSubscriptionActive;
            const isLoading = loadingPlanId === plan.id;
            const isFree = plan.id === "free";
            
            return (
              <Card 
                key={plan.id} 
                className={`flex flex-col ${plan.popular ? 'border-primary shadow-lg' : ''}`}
              >
                {plan.popular && (
                  <div className="bg-primary text-primary-foreground text-center py-1 text-sm">
                    Most Popular
                  </div>
                )}
                <CardHeader>
                  <CardTitle>{plan.name}</CardTitle>
                  <div className="mt-2">
                    <div className="text-2xl font-bold">{plan.price}</div>
                    <div className="text-sm text-muted-foreground">
                      Best for: {plan.bestFor}
                    </div>
                  </div>
                </CardHeader>
                <CardContent className="flex-grow">
                  <ul className="space-y-2">
                    {plan.features.map((feature, index) => (
                      <li key={index} className="flex items-start">
                        <Check className="h-5 w-5 text-green-500 mr-2 flex-shrink-0" />
                        <span>{feature}</span>
                      </li>
                    ))}
                  </ul>
                </CardContent>
                <CardFooter>
                  <Button
                    variant={isCurrentPlan ? "outline" : plan.popular ? "default" : "outline"}
                    className="w-full"
                    disabled={isCurrentPlan || isLoading}
                    onClick={() => handleSelectPlan(plan.id)}
                  >
                    {isLoading ? (
                      <>
                        <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                        Please wait
                      </>
                    ) : isCurrentPlan ? (
                      "Current Plan"
                    ) : plan.custom ? (
                      "Contact Us"
                    ) : isFree ? (
                      "Free Plan"
                    ) : (
                      `Subscribe for ${plan.price}`
                    )}
                  </Button>
                </CardFooter>
              </Card>
            );
          })}
        </div>
        
        <div className="mt-16 max-w-3xl mx-auto">
          <h2 className="text-2xl font-bold mb-4">Frequently Asked Questions</h2>
          <div className="space-y-6">
            <div>
              <h3 className="text-lg font-semibold">Can I upgrade or downgrade my plan?</h3>
              <p className="text-muted-foreground">
                Yes, you can change your plan at any time. When you upgrade, you'll be charged the prorated difference immediately.
                When you downgrade, your new rate will be applied at the start of your next billing cycle.
              </p>
            </div>
            <div>
              <h3 className="text-lg font-semibold">Do you offer a free trial?</h3>
              <p className="text-muted-foreground">
                We offer a free tier that includes basic functionality for one robot. This allows you to test 
                our platform before committing to a paid plan.
              </p>
            </div>
            <div>
              <h3 className="text-lg font-semibold">What happens if I exceed my robot limit?</h3>
              <p className="text-muted-foreground">
                If you approach your plan's robot limit, we'll notify you so you can upgrade to a higher tier.
                We don't automatically charge you for overages.
              </p>
            </div>
          </div>
        </div>
      </div>
    </MainLayout>
  );
};

export default Pricing;
