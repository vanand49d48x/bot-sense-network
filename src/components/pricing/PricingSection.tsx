
import React from "react";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Check } from "lucide-react";
import { Link } from "react-router-dom";

type PlanFeature = {
  text: string;
};

type PricingPlan = {
  name: string;
  description: string;
  price: string;
  bestFor: string;
  features: PlanFeature[];
  buttonText: string;
  popular?: boolean;
  buttonVariant?: "default" | "outline";
  priceId?: string; // For Stripe integration
};

const pricingPlans: PricingPlan[] = [
  {
    name: "Free",
    description: "Get started with robot monitoring",
    price: "$0",
    bestFor: "Hobbyists, solo developers",
    features: [
      { text: "1 robot" },
      { text: "3 days telemetry history" },
      { text: "Live dashboard" },
      { text: "Basic API access" },
    ],
    buttonText: "Get Started",
    buttonVariant: "outline",
  },
  {
    name: "Starter",
    description: "Ideal for small projects",
    price: "$19",
    bestFor: "Startup teams, testers",
    features: [
      { text: "Up to 5 robots" },
      { text: "7 days history" },
      { text: "Email alerts" },
      { text: "ROS/Arduino SDK support" },
    ],
    buttonText: "Subscribe",
    priceId: "price_starter123",
  },
  {
    name: "Growth",
    description: "For growing teams",
    price: "$49",
    bestFor: "Growing teams, labs",
    popular: true,
    features: [
      { text: "25 robots" },
      { text: "30-day telemetry retention" },
      { text: "Custom metrics" },
      { text: "Real-time alerts" },
      { text: "Location map" },
    ],
    buttonText: "Subscribe",
    priceId: "price_growth123",
  },
  {
    name: "Pro",
    description: "For professional deployments",
    price: "$149",
    bestFor: "Industrial use, fleets",
    features: [
      { text: "100+ robots" },
      { text: "90-day history" },
      { text: "SLA + Priority support" },
      { text: "Custom alerting rules" },
      { text: "Export + team access" },
    ],
    buttonText: "Subscribe",
    priceId: "price_pro123",
  },
  {
    name: "Enterprise",
    description: "Custom solutions for large organizations",
    price: "Custom",
    bestFor: "OEMs, manufacturers",
    features: [
      { text: "Unlimited robots" },
      { text: "Custom data retention" },
      { text: "SSO / Audit logs" },
      { text: "Dedicated onboarding" },
      { text: "On-premise / MQTT support" },
    ],
    buttonText: "Contact Sales",
    buttonVariant: "outline",
  },
];

export function PricingSection() {
  return (
    <section id="pricing" className="py-16 container mx-auto px-4">
      <div className="text-center mb-12">
        <h2 className="text-3xl md:text-4xl font-bold mb-4">Simple, Transparent Pricing</h2>
        <p className="text-xl text-muted-foreground max-w-2xl mx-auto">
          Choose the plan that's right for you. All plans include access to our core monitoring features.
        </p>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 xl:grid-cols-5 gap-6">
        {pricingPlans.map((plan) => (
          <Card 
            key={plan.name}
            className={`relative flex flex-col h-full transition-all duration-200 hover:shadow-lg hover:border-primary ${
              plan.popular ? "border-primary shadow-md" : ""
            }`}
          >
            {plan.popular && (
              <div className="absolute top-0 right-0 transform translate-x-2 -translate-y-2">
                <span className="bg-primary text-primary-foreground text-xs px-2 py-1 rounded-md">Popular</span>
              </div>
            )}
            <CardHeader>
              <CardTitle>{plan.name}</CardTitle>
              <CardDescription>{plan.bestFor}</CardDescription>
            </CardHeader>
            <CardContent className="flex-grow">
              <div className="mb-4">
                <span className="text-3xl font-bold">{plan.price}</span>
                {plan.price !== "Custom" && <span className="text-muted-foreground">/month</span>}
              </div>
              <p className="text-muted-foreground mb-6">{plan.description}</p>
              <ul className="space-y-2">
                {plan.features.map((feature, index) => (
                  <li key={index} className="flex items-center gap-2">
                    <Check className="h-4 w-4 text-green-500 flex-shrink-0" />
                    <span className="text-sm">{feature.text}</span>
                  </li>
                ))}
              </ul>
            </CardContent>
            <CardFooter>
              {plan.name === "Free" ? (
                <Button asChild className="w-full" variant={plan.buttonVariant || "default"}>
                  <Link to="/auth">{plan.buttonText}</Link>
                </Button>
              ) : plan.name === "Enterprise" ? (
                <Button asChild className="w-full" variant={plan.buttonVariant || "default"}>
                  <Link to="/contact">{plan.buttonText}</Link>
                </Button>
              ) : (
                <Button 
                  className="w-full" 
                  variant={plan.buttonVariant || "default"}
                  onClick={() => {
                    // Will be replaced with actual checkout functionality
                    if (plan.priceId) {
                      window.location.href = `/checkout?plan=${plan.name.toLowerCase()}&priceId=${plan.priceId}`;
                    }
                  }}
                >
                  {plan.buttonText}
                </Button>
              )}
            </CardFooter>
          </Card>
        ))}
      </div>

      <div className="mt-12 text-center">
        <p className="text-muted-foreground">
          All plans come with a 14-day free trial. No credit card required to start.
        </p>
      </div>
    </section>
  );
}
