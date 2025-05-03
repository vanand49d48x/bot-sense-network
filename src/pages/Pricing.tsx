
import React, { useState } from "react";
import { MainLayout } from "@/components/layout/MainLayout";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Check } from "lucide-react";
import { useToast } from "@/hooks/use-toast";
import { useNavigate } from "react-router-dom";
import { useAuth } from "@/context/AuthContext";
import { supabase } from "@/integrations/supabase/client";

interface PricingPlan {
  title: string;
  price: string;
  description: string;
  features: string[];
  buttonText: string;
  popular?: boolean;
  priceId: string;
}

const pricingPlans: PricingPlan[] = [
  {
    title: "Free",
    price: "$0",
    description: "For individual robot enthusiasts",
    features: [
      "Monitor up to 3 robots",
      "Basic telemetry data",
      "24-hour data retention",
      "Email alerts"
    ],
    buttonText: "Get Started",
    priceId: "free"
  },
  {
    title: "Pro",
    price: "$29",
    description: "For small teams and businesses",
    features: [
      "Monitor up to 25 robots",
      "Advanced telemetry data",
      "30-day data retention",
      "Email and SMS alerts",
      "API access",
      "Custom robot types"
    ],
    buttonText: "Subscribe",
    popular: true,
    priceId: "price_pro"
  },
  {
    title: "Enterprise",
    price: "$99",
    description: "For large organizations",
    features: [
      "Unlimited robots",
      "Advanced telemetry data",
      "90-day data retention",
      "Priority support",
      "Custom integrations",
      "Dedicated account manager",
      "On-premise deployment option"
    ],
    buttonText: "Contact Sales",
    priceId: "price_enterprise"
  }
];

export default function Pricing() {
  const { toast } = useToast();
  const navigate = useNavigate();
  const { user } = useAuth();
  const [loading, setLoading] = useState<Record<string, boolean>>({});

  const handleSubscription = async (plan: PricingPlan) => {
    if (plan.title === "Free") {
      navigate("/dashboard");
      return;
    }
    
    if (plan.title === "Enterprise") {
      toast({
        title: "Enterprise Plan",
        description: "Please contact our sales team for enterprise pricing.",
      });
      return;
    }

    if (!user) {
      toast({
        title: "Authentication Required",
        description: "Please sign in to subscribe to this plan.",
      });
      navigate("/auth");
      return;
    }

    try {
      setLoading(prev => ({ ...prev, [plan.priceId]: true }));
      
      const { data, error } = await supabase.functions.invoke('create-checkout', {
        body: { priceId: plan.priceId }
      });
      
      if (error) throw error;
      
      if (data?.url) {
        window.location.href = data.url;
      } else {
        throw new Error("No checkout URL returned");
      }
    } catch (error) {
      console.error("Error creating checkout session:", error);
      toast({
        title: "Checkout Error",
        description: "There was an error starting the checkout process. Please try again.",
        variant: "destructive",
      });
    } finally {
      setLoading(prev => ({ ...prev, [plan.priceId]: false }));
    }
  };

  return (
    <div className="bg-background">
      <header className="bg-muted/40 py-12">
        <div className="container mx-auto px-4 text-center">
          <h1 className="text-4xl font-bold tracking-tight mb-4">Simple, transparent pricing</h1>
          <p className="text-lg text-muted-foreground max-w-2xl mx-auto">
            Choose the perfect plan for your robot monitoring needs. All plans include our core monitoring features.
          </p>
        </div>
      </header>

      <div className="container mx-auto px-4 py-16">
        <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
          {pricingPlans.map((plan) => (
            <Card key={plan.title} className={`flex flex-col ${plan.popular ? 'border-primary shadow-lg' : ''}`}>
              <CardHeader>
                <div className="flex justify-between items-center">
                  <CardTitle className="text-xl">{plan.title}</CardTitle>
                  {plan.popular && <Badge variant="default">Popular</Badge>}
                </div>
                <CardDescription>{plan.description}</CardDescription>
                <div className="mt-4">
                  <span className="text-3xl font-bold">{plan.price}</span>
                  {plan.title !== "Free" && <span className="text-muted-foreground">/month</span>}
                </div>
              </CardHeader>
              
              <CardContent className="flex-grow">
                <ul className="space-y-2 mb-6">
                  {plan.features.map((feature, index) => (
                    <li key={index} className="flex items-center gap-2">
                      <Check className="h-4 w-4 text-primary flex-shrink-0" />
                      <span>{feature}</span>
                    </li>
                  ))}
                </ul>
              </CardContent>
              
              <CardFooter>
                <Button 
                  className="w-full" 
                  onClick={() => handleSubscription(plan)}
                  disabled={loading[plan.priceId]}
                  variant={plan.popular ? "default" : "outline"}
                >
                  {loading[plan.priceId] ? "Processing..." : plan.buttonText}
                </Button>
              </CardFooter>
            </Card>
          ))}
        </div>

        <div className="mt-16 text-center">
          <h2 className="text-2xl font-bold mb-4">Questions about our pricing?</h2>
          <p className="text-muted-foreground max-w-2xl mx-auto mb-6">
            Our support team is here to help with your questions about our plans and pricing.
          </p>
          <Button variant="outline" onClick={() => navigate("/contact")}>
            Contact Support
          </Button>
        </div>
      </div>
    </div>
  );
}
