
import React, { useEffect, useState } from "react";
import { Link, useNavigate } from "react-router-dom";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Check, ChevronRight, Loader2 } from "lucide-react";
import { useAuth } from "@/context/AuthContext";
import { supabase } from "@/integrations/supabase/client";
import { useToast } from "@/hooks/use-toast";

const CheckoutSuccess = () => {
  const { user } = useAuth();
  const navigate = useNavigate();
  const { toast } = useToast();
  const [isLoading, setIsLoading] = useState(true);
  const [subscriptionDetails, setSubscriptionDetails] = useState<any>(null);

  useEffect(() => {
    if (!user) {
      // If no user is logged in, redirect to login
      navigate("/auth");
      return;
    }

    // Verify subscription status
    const checkSubscription = async () => {
      try {
        setIsLoading(true);
        
        // Call the subscription check edge function
        const { data, error } = await supabase.functions.invoke('check-subscription');
        
        if (error) {
          throw error;
        }
        
        console.log("Subscription check result:", data);
        setSubscriptionDetails(data);
        
        if (data.active) {
          toast({
            title: "Subscription Active",
            description: `Your ${data.plan} plan is now active!`,
            variant: "default",
          });
        } else {
          toast({
            title: "Subscription Pending",
            description: "Your subscription is being processed. This may take a moment.",
            variant: "default",
          });
          
          // Try again in 5 seconds
          setTimeout(checkSubscription, 5000);
        }
      } catch (error) {
        console.error("Error checking subscription:", error);
        toast({
          title: "Error",
          description: "Unable to verify your subscription status. Please check your account details.",
          variant: "destructive",
        });
      } finally {
        setIsLoading(false);
      }
    };

    checkSubscription();
  }, [user, navigate, toast]);

  return (
    <div className="min-h-screen bg-background flex flex-col items-center justify-center p-4">
      <Card className="max-w-md w-full">
        <CardHeader className="text-center">
          <div className="mx-auto bg-green-100 text-green-600 p-3 rounded-full w-16 h-16 flex items-center justify-center mb-4">
            {isLoading ? (
              <Loader2 className="h-8 w-8 animate-spin" />
            ) : (
              <Check className="h-8 w-8" />
            )}
          </div>
          <CardTitle className="text-2xl">Payment Successful!</CardTitle>
          <CardDescription>Thank you for subscribing to RoboMetrics</CardDescription>
        </CardHeader>
        <CardContent className="text-center">
          {isLoading ? (
            <p className="mb-4">
              Verifying your subscription status...
            </p>
          ) : subscriptionDetails?.active ? (
            <div className="space-y-2">
              <p className="mb-4">
                Your subscription to the <span className="font-semibold">{subscriptionDetails.plan}</span> plan has been activated successfully.
              </p>
              <p>
                You now have access to all features included in your plan until {new Date(subscriptionDetails.subscription_end).toLocaleDateString()}.
              </p>
            </div>
          ) : (
            <p className="mb-4">
              Your payment was successful. Your subscription will be activated shortly.
            </p>
          )}
        </CardContent>
        <CardFooter className="flex flex-col gap-2">
          <Button asChild className="w-full">
            <Link to="/dashboard">
              Go to Dashboard
              <ChevronRight className="ml-2 h-4 w-4" />
            </Link>
          </Button>
          <Button asChild variant="outline" className="w-full">
            <Link to="/profile">
              Manage Subscription
            </Link>
          </Button>
        </CardFooter>
      </Card>
    </div>
  );
};

export default CheckoutSuccess;
