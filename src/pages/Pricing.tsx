
import React, { useEffect, useState } from "react";
import { Link, useNavigate, useSearchParams } from "react-router-dom";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Bot, Check, ChevronRight, Shield, Zap } from "lucide-react";
import { useAuth } from "@/context/AuthContext";
import { supabase } from "@/integrations/supabase/client";
import { useToast } from "@/hooks/use-toast";

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

const Pricing = () => {
  const { user } = useAuth();
  const { toast } = useToast();
  const navigate = useNavigate();
  const [searchParams] = useSearchParams();
  const [isRedirecting, setIsRedirecting] = useState(false);
  
  // Check for plans in URL params to handle post-login returns
  const pendingPriceId = searchParams.get('priceId');
  
  // Effect to handle subscription after login
  useEffect(() => {
    const handlePostLoginSubscription = async () => {
      if (user && pendingPriceId && !isRedirecting) {
        setIsRedirecting(true);
        
        try {
          toast({
            title: "Redirecting to checkout",
            description: "Please wait while we prepare your checkout session",
          });
          
          const { data, error } = await supabase.functions.invoke('create-checkout', {
            body: { priceId: pendingPriceId }
          });
          
          if (error) {
            throw error;
          }
          
          // Clear the URL parameter to prevent continuous redirects
          window.history.replaceState({}, document.title, window.location.pathname);
          
          // Redirect to Stripe Checkout
          window.location.href = data.url;
        } catch (error: any) {
          console.error("Error creating checkout session:", error);
          toast({
            title: "Error",
            description: error.message || "Failed to create checkout session",
            variant: "destructive",
          });
          setIsRedirecting(false);
          
          // Clear the URL parameter even on error to prevent continuous retries
          window.history.replaceState({}, document.title, window.location.pathname);
        }
      }
    };
    
    handlePostLoginSubscription();
  }, [user, pendingPriceId, isRedirecting, toast]);

  const handleSubscription = async (plan: Plan) => {
    if (plan.id === "free") {
      // Navigate to signup for free plan
      window.location.href = "/auth?tab=signup";
      return;
    }

    if (plan.id === "enterprise") {
      // Navigate to contact page for enterprise plan
      window.location.href = "/contact";
      return;
    }

    try {
      if (!user) {
        // Not logged in - redirect to auth page with return URL to pricing and the priceId
        navigate(`/auth?returnUrl=/pricing&priceId=${plan.priceId}`);
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

  return (
    <div className="min-h-screen bg-background">
      {/* Header */}
      <header className="border-b border-border/40 p-4">
        <div className="container mx-auto flex justify-between items-center">
          <div className="flex items-center gap-2">
            <Bot className="h-6 w-6 text-primary" />
            <Link to="/" className="text-xl font-bold">RoboMetrics</Link>
          </div>
          <div className="flex items-center gap-4">
            {!user ? (
              <>
                <Link to="/auth" className="text-muted-foreground hover:text-foreground transition-colors">
                  Login
                </Link>
                <Link to="/auth?tab=signup" className="text-muted-foreground hover:text-foreground transition-colors">
                  Sign Up
                </Link>
              </>
            ) : (
              <Link to="/dashboard" className="text-muted-foreground hover:text-foreground transition-colors">
                Dashboard
              </Link>
            )}
          </div>
        </div>
      </header>

      {/* Hero Section */}
      <section className="py-16 container mx-auto px-4 text-center">
        <h1 className="text-4xl md:text-5xl font-bold mb-4">Simple, Transparent Pricing</h1>
        <p className="text-xl text-muted-foreground mb-8 max-w-2xl mx-auto">
          Choose the plan that fits your robot monitoring needs
        </p>
      </section>

      {/* Pricing Section */}
      <section className="py-8 container mx-auto px-4 mb-16">
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-5 gap-6">
          {plans.map((plan) => (
            <Card 
              key={plan.id}
              className={`flex flex-col ${plan.highlight ? 'border-primary shadow-lg' : ''}`}
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
                  {plan.id !== "free" && plan.id !== "enterprise" && <span className="text-muted-foreground">/month</span>}
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
      </section>

      {/* FAQ Section */}
      <section className="py-16 bg-muted/30">
        <div className="container mx-auto px-4">
          <h2 className="text-3xl font-bold mb-8 text-center">Frequently Asked Questions</h2>
          <div className="grid md:grid-cols-2 gap-8 max-w-4xl mx-auto">
            <div>
              <h3 className="text-xl font-semibold mb-2">Can I upgrade or downgrade my plan?</h3>
              <p className="text-muted-foreground">Yes, you can upgrade or downgrade your plan at any time. Changes will take effect at the start of your next billing cycle.</p>
            </div>
            <div>
              <h3 className="text-xl font-semibold mb-2">How does billing work?</h3>
              <p className="text-muted-foreground">We bill monthly or annually, depending on your preference. All plans are auto-renewed unless cancelled.</p>
            </div>
            <div>
              <h3 className="text-xl font-semibold mb-2">Do you offer refunds?</h3>
              <p className="text-muted-foreground">We offer a 30-day money-back guarantee for all paid plans. If you're not satisfied, contact our support team.</p>
            </div>
            <div>
              <h3 className="text-xl font-semibold mb-2">What happens if I exceed my robot limit?</h3>
              <p className="text-muted-foreground">You'll need to upgrade to a higher tier plan that accommodates your needs. We'll notify you when you're approaching your limit.</p>
            </div>
          </div>
        </div>
      </section>

      {/* CTA Section */}
      <section className="py-16 bg-primary/5">
        <div className="container mx-auto px-4 text-center">
          <h2 className="text-3xl font-bold mb-4">Ready to get started?</h2>
          <p className="text-xl mb-8 max-w-2xl mx-auto">
            Try RoboMetrics free for 7 days, no credit card required.
          </p>
          <div className="flex flex-col sm:flex-row gap-4 justify-center">
            <Link to="/auth?tab=signup">
              <Button size="lg" className="px-8 py-6 text-lg">
                Start Free Trial
              </Button>
            </Link>
            <Link to="/contact">
              <Button size="lg" variant="outline" className="px-8 py-6 text-lg">
                Contact Sales
              </Button>
            </Link>
          </div>
        </div>
      </section>
    </div>
  );
};

export default Pricing;
