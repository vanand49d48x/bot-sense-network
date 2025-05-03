
import React, { useState } from "react";
import { Button } from "@/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardFooter,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Check, Info, AlertCircle } from "lucide-react";
import { useNavigate } from "react-router-dom";
import { useAuth } from "@/context/AuthContext";
import { useToast } from "@/components/ui/use-toast";
import { Tooltip, TooltipContent, TooltipProvider, TooltipTrigger } from "@/components/ui/tooltip";
import PlaceholderLayout from "@/components/layout/PlaceholderLayout";
import { supabase } from "@/integrations/supabase/client";

type PlanFeature = {
  name: string;
  included: boolean;
  info?: string;
};

type PricingPlan = {
  id: string;
  name: string;
  price: number;
  priceId: string | null;
  description: string;
  features: PlanFeature[];
  popular?: boolean;
  priceDisplay: string;
  bestFor: string;
};

const Pricing = () => {
  const { user } = useAuth();
  const navigate = useNavigate();
  const { toast } = useToast();
  const [loadingPlan, setLoadingPlan] = useState<string | null>(null);

  const plans: PricingPlan[] = [
    {
      id: "free",
      name: "Free",
      price: 0,
      priceId: null,
      priceDisplay: "$0/month",
      bestFor: "Hobbyists, solo developers",
      description: "Perfect for getting started with robot monitoring",
      features: [
        { name: "1 robot", included: true },
        { name: "3 days telemetry history", included: true },
        { name: "Live dashboard", included: true },
        { name: "Basic API access", included: true },
        { name: "Email alerts", included: false },
        { name: "ROS/Arduino SDK support", included: false },
        { name: "Custom metrics", included: false },
        { name: "Location map", included: false },
      ],
    },
    {
      id: "starter",
      name: "Starter",
      price: 19,
      priceId: "STRIPE_STARTER_PRICE_ID",
      priceDisplay: "$19/month",
      bestFor: "Startup teams, testers",
      description: "For teams starting with robot fleets",
      features: [
        { name: "Up to 5 robots", included: true },
        { name: "7 days history", included: true },
        { name: "Email alerts", included: true },
        { name: "ROS/Arduino SDK support", included: true },
        { name: "Custom metrics", included: false },
        { name: "Real-time alerts", included: false },
        { name: "Location map", included: false },
        { name: "SLA + Priority support", included: false },
      ],
    },
    {
      id: "growth",
      name: "Growth",
      price: 49,
      priceId: "STRIPE_GROWTH_PRICE_ID",
      priceDisplay: "$49/month",
      bestFor: "Growing teams, labs",
      description: "For teams with growing robot fleets",
      popular: true,
      features: [
        { name: "25 robots", included: true },
        { name: "30-day telemetry retention", included: true },
        { name: "Custom metrics", included: true },
        { name: "Real-time alerts", included: true },
        { name: "Location map", included: true },
        { name: "Export + team access", included: false },
        { name: "SLA + Priority support", included: false },
        { name: "Custom alerting rules", included: false },
      ],
    },
    {
      id: "pro",
      name: "Pro",
      price: 149,
      priceId: "STRIPE_PRO_PRICE_ID",
      priceDisplay: "$149/month",
      bestFor: "Industrial use, fleets",
      description: "For large-scale robot fleet operations",
      features: [
        { name: "100+ robots", included: true },
        { name: "90-day history", included: true },
        { name: "SLA + Priority support", included: true },
        { name: "Custom alerting rules", included: true },
        { name: "Export + team access", included: true },
        { name: "Custom metrics", included: true },
        { name: "Real-time alerts", included: true },
        { name: "Location map", included: true },
      ],
    },
  ];

  const handlePlanSelect = async (plan: PricingPlan) => {
    if (plan.id === "free") {
      navigate("/dashboard");
      return;
    }

    if (!user) {
      toast({
        title: "Sign in required",
        description: "Please sign in or create an account to subscribe",
        variant: "destructive",
      });
      navigate("/auth?tab=signin&redirect=pricing");
      return;
    }

    try {
      setLoadingPlan(plan.id);
      
      const { data, error } = await supabase.functions.invoke("create-checkout", {
        body: { priceId: plan.priceId }
      });

      if (error) {
        throw new Error(error.message);
      }

      if (data?.url) {
        window.location.href = data.url;
      } else {
        throw new Error("No checkout URL returned");
      }
    } catch (error) {
      console.error("Error creating checkout session:", error);
      toast({
        title: "Checkout Error",
        description: "Failed to create checkout session. Please try again.",
        variant: "destructive",
      });
    } finally {
      setLoadingPlan(null);
    }
  };

  return (
    <PlaceholderLayout title="Pricing Plans">
      <div className="max-w-7xl mx-auto pb-16">
        <div className="text-center mb-12">
          <h2 className="text-3xl font-bold mb-4">Choose the Right Plan for Your Robots</h2>
          <p className="text-muted-foreground max-w-3xl mx-auto">
            From hobbyist projects to industrial-scale deployments, we have a plan that fits your needs.
            All plans include our core monitoring features and reliable infrastructure.
          </p>
        </div>

        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-8">
          {plans.map((plan) => (
            <Card 
              key={plan.id} 
              className={`flex flex-col ${
                plan.popular 
                  ? "border-primary shadow-lg relative" 
                  : "border-border"
              }`}
            >
              {plan.popular && (
                <Badge 
                  variant="default" 
                  className="absolute top-0 right-0 transform translate-x-1/4 -translate-y-1/2"
                >
                  Most Popular
                </Badge>
              )}
              <CardHeader>
                <CardTitle>{plan.name}</CardTitle>
                <CardDescription>{plan.bestFor}</CardDescription>
              </CardHeader>
              <CardContent className="flex-grow">
                <div className="mb-6">
                  <p className="text-3xl font-bold">{plan.priceDisplay}</p>
                  <p className="text-muted-foreground text-sm">{plan.description}</p>
                </div>

                <div className="space-y-3">
                  {plan.features.map((feature, index) => (
                    <div key={index} className="flex items-start gap-2">
                      {feature.included ? (
                        <Check className="h-5 w-5 text-green-500 mt-0.5 flex-shrink-0" />
                      ) : (
                        <div className="h-5 w-5 flex items-center justify-center flex-shrink-0">
                          <span className="block h-0.5 w-2 bg-muted-foreground rounded-full"></span>
                        </div>
                      )}
                      <div className="flex items-center gap-1">
                        <span className={feature.included ? "" : "text-muted-foreground"}>
                          {feature.name}
                        </span>
                        {feature.info && (
                          <TooltipProvider>
                            <Tooltip>
                              <TooltipTrigger asChild>
                                <Info className="h-3.5 w-3.5 text-muted-foreground cursor-help" />
                              </TooltipTrigger>
                              <TooltipContent>
                                {feature.info}
                              </TooltipContent>
                            </Tooltip>
                          </TooltipProvider>
                        )}
                      </div>
                    </div>
                  ))}
                </div>
              </CardContent>
              <CardFooter>
                <Button 
                  className="w-full" 
                  variant={plan.popular ? "default" : "outline"}
                  onClick={() => handlePlanSelect(plan)}
                  disabled={loadingPlan === plan.id}
                >
                  {loadingPlan === plan.id ? (
                    "Processing..."
                  ) : plan.id === "free" ? (
                    "Get Started"
                  ) : (
                    `Subscribe to ${plan.name}`
                  )}
                </Button>
              </CardFooter>
            </Card>
          ))}
        </div>
        
        <div className="mt-16 flex justify-center">
          <Card className="max-w-2xl w-full">
            <CardHeader className="flex flex-row items-center gap-4">
              <AlertCircle className="h-6 w-6 text-primary" />
              <div>
                <CardTitle>Need a custom plan?</CardTitle>
                <CardDescription>For enterprises with specific requirements</CardDescription>
              </div>
            </CardHeader>
            <CardContent>
              <p className="text-muted-foreground">
                If you need custom integrations, more robots, longer retention periods, or other custom features,
                we offer tailored enterprise plans to meet your specific requirements.
              </p>
            </CardContent>
            <CardFooter>
              <Button variant="outline" className="w-full" onClick={() => navigate('/contact')}>
                Contact Sales
              </Button>
            </CardFooter>
          </Card>
        </div>
      </div>
    </PlaceholderLayout>
  );
};

export default Pricing;
