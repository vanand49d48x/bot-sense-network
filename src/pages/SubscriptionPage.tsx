
import { useState, useEffect } from "react";
import { MainLayout } from "@/components/layout/MainLayout";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Link, useSearchParams } from "react-router-dom";
import { Badge } from "@/components/ui/badge";
import { useAuth } from "@/context/AuthContext";
import { CreditCard, Zap, Check, ChevronRight, AlertCircle, RefreshCw } from "lucide-react";
import { toast } from "@/components/ui/sonner";
import { supabase } from "@/integrations/supabase/client";

export default function SubscriptionPage() {
  const { user } = useAuth();
  const [searchParams] = useSearchParams();
  const [currentPlan, setCurrentPlan] = useState("Free");
  const [isLoading, setIsLoading] = useState(false);
  const [subscriptionEnd, setSubscriptionEnd] = useState<string | null>(null);
  const [customerId, setCustomerId] = useState<string | null>(null);
  
  const success = searchParams.get("success");
  
  useEffect(() => {
    if (success) {
      toast.success("Payment successful! Your subscription is now active.");
    }
  }, [success]);

  // Fetch the user's subscription status
  useEffect(() => {
    if (!user) return;

    const checkSubscription = async () => {
      setIsLoading(true);
      try {
        const { data, error } = await supabase.functions.invoke('check-subscription');
        
        if (error) {
          throw error;
        }
        
        if (data.subscribed) {
          setCurrentPlan(data.subscription_tier);
          setSubscriptionEnd(data.subscription_end);
          setCustomerId(data.customer_id);
        } else {
          setCurrentPlan("Free");
          setSubscriptionEnd(null);
          setCustomerId(data.customer_id);
        }
      } catch (err) {
        console.error("Error checking subscription:", err);
        toast.error("Failed to check subscription status.");
      } finally {
        setIsLoading(false);
      }
    };

    checkSubscription();
  }, [user]);

  const handleManageSubscription = async () => {
    if (!customerId) {
      toast.error("No subscription found to manage");
      return;
    }

    setIsLoading(true);
    try {
      // Implement customer portal creation here
      toast.info("Redirecting to billing management...");
      setTimeout(() => {
        setIsLoading(false);
      }, 1000);
    } catch (err) {
      console.error("Error creating customer portal:", err);
      toast.error("Failed to redirect to billing portal");
      setIsLoading(false);
    }
  };

  const handleCheckSubscription = async () => {
    if (!user) return;
    
    setIsLoading(true);
    try {
      const { data, error } = await supabase.functions.invoke('check-subscription');
      
      if (error) {
        throw error;
      }
      
      if (data.subscribed) {
        setCurrentPlan(data.subscription_tier);
        setSubscriptionEnd(data.subscription_end);
        setCustomerId(data.customer_id);
        toast.success(`Subscription updated: ${data.subscription_tier} plan`);
      } else {
        setCurrentPlan("Free");
        setSubscriptionEnd(null);
        setCustomerId(data.customer_id);
        toast.info("No active subscription found");
      }
    } catch (err) {
      console.error("Error checking subscription:", err);
      toast.error("Failed to check subscription status");
    } finally {
      setIsLoading(false);
    }
  };
  
  const plans = [
    {
      name: "Free",
      price: "$0",
      period: "month",
      description: "For hobbyists and solo developers",
      features: [
        "1 robot",
        "3 days telemetry history",
        "Live dashboard",
        "Basic API access"
      ],
      isCurrent: currentPlan === "Free",
      buttonText: currentPlan === "Free" ? "Current Plan" : "Downgrade",
      buttonVariant: currentPlan === "Free" ? "outline" : "secondary"
    },
    {
      name: "Starter",
      price: "$19",
      period: "month",
      description: "For startup teams and testers",
      features: [
        "Up to 5 robots",
        "7 days history",
        "Email alerts",
        "ROS/Arduino SDK support"
      ],
      isCurrent: currentPlan === "Starter",
      buttonText: currentPlan === "Starter" ? "Current Plan" : 
                 (currentPlan === "Free" ? "Upgrade" : "Downgrade"),
      buttonVariant: currentPlan === "Starter" ? "outline" : 
                    (currentPlan === "Free" ? "default" : "secondary")
    },
    {
      name: "Growth",
      price: "$49",
      period: "month",
      description: "For growing teams and labs",
      features: [
        "25 robots",
        "30-day telemetry retention",
        "Custom metrics",
        "Real-time alerts",
        "Location map"
      ],
      isCurrent: currentPlan === "Growth",
      buttonText: currentPlan === "Growth" ? "Current Plan" : 
                 (["Free", "Starter"].includes(currentPlan) ? "Upgrade" : "Downgrade"),
      buttonVariant: currentPlan === "Growth" ? "outline" : 
                    (["Free", "Starter"].includes(currentPlan) ? "default" : "secondary"),
      popular: true
    },
    {
      name: "Pro",
      price: "$149",
      period: "month",
      description: "For industrial use and fleets",
      features: [
        "100+ robots",
        "90-day history",
        "SLA + Priority support",
        "Custom alerting rules",
        "Export + team access"
      ],
      isCurrent: currentPlan === "Pro",
      buttonText: currentPlan === "Pro" ? "Current Plan" : "Upgrade",
      buttonVariant: currentPlan === "Pro" ? "outline" : "default"
    }
  ];

  const handleSubscription = (plan: string) => {
    if (plan === currentPlan) {
      toast.info("You are already subscribed to this plan");
      return;
    }
    
    // Redirect to pricing page
    window.location.href = "/pricing";
  };

  const addOns = [
    {
      name: "Additional telemetry storage",
      description: "$5/month per extra 30 days",
      icon: <Zap className="h-5 w-5 text-blue-500" />
    },
    {
      name: "Extra robot",
      description: "$2-3/month per bot on Free/Starter plans",
      icon: <Zap className="h-5 w-5 text-blue-500" />
    },
    {
      name: "Team seats",
      description: "$10/month per extra admin",
      icon: <Zap className="h-5 w-5 text-blue-500" />
    }
  ];

  return (
    <MainLayout>
      <div className="container py-6">
        <h1 className="text-3xl font-bold mb-2">Subscription</h1>
        <p className="text-muted-foreground mb-6">Manage your subscription and add-ons</p>
        
        {isLoading ? (
          <div className="flex justify-center py-10">
            <div className="text-center">
              <div className="animate-spin mb-4 h-8 w-8 border-4 border-primary border-t-transparent rounded-full mx-auto"></div>
              <p>Loading subscription details...</p>
            </div>
          </div>
        ) : (
          <>
            <Card className="mb-8">
              <CardHeader>
                <div className="flex items-center justify-between">
                  <div>
                    <CardTitle>Current Subscription</CardTitle>
                    <CardDescription>Your active subscription details</CardDescription>
                  </div>
                  <CreditCard className="h-6 w-6 text-muted-foreground" />
                </div>
              </CardHeader>
              <CardContent>
                <div className="flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4">
                  <div>
                    <h3 className="text-lg font-medium">{currentPlan} Plan</h3>
                    <p className="text-muted-foreground">
                      {
                        currentPlan === "Free" ? "Free access to basic features" :
                        subscriptionEnd ? `Active until ${new Date(subscriptionEnd).toLocaleDateString()}` : 
                        "Subscription active"
                      }
                    </p>
                  </div>
                  <div className="flex gap-3">
                    <Button 
                      variant="outline" 
                      className="flex items-center gap-2" 
                      onClick={handleCheckSubscription}
                      disabled={isLoading}
                    >
                      <RefreshCw className="h-4 w-4" />
                      Refresh Status
                    </Button>
                    <Link to="/pricing">
                      <Button variant="outline">View All Plans</Button>
                    </Link>
                    {currentPlan !== "Free" && customerId && (
                      <Button variant="default" onClick={handleManageSubscription} disabled={isLoading}>
                        Manage Billing
                      </Button>
                    )}
                  </div>
                </div>
                {currentPlan === "Free" && (
                  <div className="mt-4 bg-muted/50 p-3 rounded-lg flex items-start gap-3">
                    <AlertCircle className="h-5 w-5 text-amber-500 mt-0.5" />
                    <div>
                      <h4 className="font-medium">You're on the Free plan</h4>
                      <p className="text-sm text-muted-foreground">
                        Upgrade to access additional robots, longer history retention, and premium features.
                      </p>
                    </div>
                  </div>
                )}
              </CardContent>
            </Card>
            
            <h2 className="text-2xl font-bold mb-4">Available Plans</h2>
            <div className="grid gap-6 md:grid-cols-2 lg:grid-cols-4 mb-8">
              {plans.map((plan) => (
                <Card 
                  key={plan.name}
                  className={`flex flex-col transition-all hover:border-primary hover:shadow-md ${
                    plan.isCurrent ? "border-primary shadow-sm" : ""
                  } ${plan.popular ? "relative" : ""}`}
                >
                  {plan.popular && (
                    <div className="absolute -top-3 left-0 right-0 flex justify-center">
                      <Badge className="bg-primary hover:bg-primary">Most Popular</Badge>
                    </div>
                  )}
                  <CardHeader>
                    <CardTitle>{plan.name}</CardTitle>
                    <div className="mt-2">
                      <span className="text-3xl font-bold">{plan.price}</span>
                      <span className="text-muted-foreground">/{plan.period}</span>
                    </div>
                    <CardDescription>{plan.description}</CardDescription>
                  </CardHeader>
                  <CardContent className="flex-grow">
                    <ul className="space-y-2">
                      {plan.features.map((feature, i) => (
                        <li key={i} className="flex items-center">
                          <Check className="h-4 w-4 mr-2 text-green-500" />
                          <span>{feature}</span>
                        </li>
                      ))}
                    </ul>
                  </CardContent>
                  <CardFooter>
                    <Button
                      className="w-full"
                      variant={plan.buttonVariant as "default" | "secondary" | "outline"}
                      disabled={plan.isCurrent}
                      onClick={() => handleSubscription(plan.name)}
                    >
                      {plan.buttonText}
                    </Button>
                  </CardFooter>
                </Card>
              ))}
            </div>
            
            <h2 className="text-2xl font-bold mb-4">Add-ons</h2>
            <div className="grid gap-6 md:grid-cols-3">
              {addOns.map((addon, i) => (
                <Card key={i} className="hover:border-primary transition-all">
                  <CardHeader className="pb-2">
                    <div className="flex items-center justify-between">
                      <CardTitle className="text-lg">{addon.name}</CardTitle>
                      {addon.icon}
                    </div>
                  </CardHeader>
                  <CardContent>
                    <p className="text-muted-foreground">{addon.description}</p>
                  </CardContent>
                  <CardFooter>
                    <Link to="/pricing" className="w-full">
                      <Button variant="outline" className="w-full">
                        Add to Cart
                      </Button>
                    </Link>
                  </CardFooter>
                </Card>
              ))}
            </div>
          </>
        )}
      </div>
    </MainLayout>
  );
}
