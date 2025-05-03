
import { useState } from "react";
import { useNavigate, useSearchParams } from "react-router-dom";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { useToast } from "@/components/ui/use-toast";
import { Loader2 } from "lucide-react";
import { supabase } from "@/integrations/supabase/client";
import { useAuth } from "@/context/AuthContext";

const planDetails = {
  starter: {
    name: "Starter Plan",
    price: "$19/month",
    description: "Perfect for startup teams and testers",
    features: [
      "Up to 5 robots",
      "7 days history",
      "Email alerts",
      "ROS/Arduino SDK support"
    ]
  },
  growth: {
    name: "Growth Plan",
    price: "$49/month",
    description: "Ideal for growing teams and labs",
    features: [
      "25 robots",
      "30-day telemetry retention",
      "Custom metrics",
      "Real-time alerts",
      "Location map"
    ]
  },
  pro: {
    name: "Pro Plan",
    price: "$149/month",
    description: "For industrial use and fleet management",
    features: [
      "100+ robots",
      "90-day history",
      "SLA + Priority support",
      "Custom alerting rules",
      "Export + team access"
    ]
  }
};

export default function CheckoutPage() {
  const [searchParams] = useSearchParams();
  const plan = searchParams.get("plan") || "";
  const priceId = searchParams.get("priceId") || "";
  
  const { toast } = useToast();
  const navigate = useNavigate();
  const { user } = useAuth();
  const [isLoading, setIsLoading] = useState(false);

  // Get plan details with fallback
  const planInfo = planDetails[plan as keyof typeof planDetails] || {
    name: "Custom Plan",
    price: "Custom price",
    description: "Please contact us for details",
    features: ["Custom features based on your needs"]
  };

  const handleCheckout = async () => {
    if (!priceId) {
      toast({
        title: "Error",
        description: "Invalid plan selected",
        variant: "destructive"
      });
      return;
    }

    try {
      setIsLoading(true);
      
      // This is where we would call the Supabase Edge Function to create a Stripe checkout session
      // For now, we'll simulate the redirect with a delay
      
      // Commented out for now until we set up the actual Stripe integration
      /*
      const { data, error } = await supabase.functions.invoke('create-checkout', {
        body: { priceId }
      });

      if (error) throw error;
      if (!data.url) throw new Error('No checkout URL returned');

      // Redirect to Stripe Checkout
      window.location.href = data.url;
      */

      // Simulating the process for now
      setTimeout(() => {
        toast({
          title: "Checkout simulation",
          description: "In a real implementation, you would be redirected to Stripe. Redirecting to success page...",
        });
        
        // Redirect to success page after a delay
        setTimeout(() => {
          navigate("/checkout-success?plan=" + plan);
        }, 2000);
      }, 1500);

    } catch (error) {
      console.error("Checkout error:", error);
      toast({
        title: "Checkout Error",
        description: error instanceof Error ? error.message : "Failed to initiate checkout",
        variant: "destructive"
      });
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="min-h-screen flex items-center justify-center bg-background p-4">
      <Card className="w-full max-w-md">
        <CardHeader>
          <CardTitle className="text-2xl">Complete Your Subscription</CardTitle>
          <CardDescription>You're subscribing to our {planInfo.name}</CardDescription>
        </CardHeader>
        <CardContent className="space-y-4">
          <div className="p-4 bg-muted rounded-lg">
            <h3 className="font-semibold text-lg">{planInfo.name}</h3>
            <p className="text-2xl font-bold mb-2">{planInfo.price}</p>
            <p className="text-sm text-muted-foreground mb-4">{planInfo.description}</p>
            
            <div className="space-y-2">
              <p className="text-sm font-medium">Plan includes:</p>
              <ul className="space-y-1">
                {planInfo.features.map((feature, index) => (
                  <li key={index} className="text-sm flex items-start gap-2">
                    <span className="text-primary">✓</span>
                    <span>{feature}</span>
                  </li>
                ))}
              </ul>
            </div>
          </div>

          {!user && (
            <div className="rounded border border-amber-200 bg-amber-50 dark:bg-amber-950/20 dark:border-amber-900 p-3 text-sm">
              <p className="font-medium text-amber-800 dark:text-amber-300">You're not logged in</p>
              <p className="mt-1 text-amber-700 dark:text-amber-400">
                For a better checkout experience, we recommend 
                <Button variant="link" className="p-0 h-auto text-amber-700 underline ml-1" 
                  onClick={() => navigate("/auth?redirect=checkout")}>
                  logging in
                </Button> first.
              </p>
            </div>
          )}
        </CardContent>
        <CardFooter className="flex flex-col gap-4">
          <Button 
            className="w-full" 
            onClick={handleCheckout}
            disabled={isLoading}
          >
            {isLoading ? (
              <>
                <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                Processing...
              </>
            ) : (
              "Proceed to Payment"
            )}
          </Button>
          <Button 
            variant="outline" 
            className="w-full"
            onClick={() => navigate("/")}
          >
            Cancel
          </Button>
        </CardFooter>
      </Card>
    </div>
  );
}
