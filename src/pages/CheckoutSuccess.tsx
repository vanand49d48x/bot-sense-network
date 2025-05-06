
import React, { useEffect } from "react";
import { Link, useNavigate } from "react-router-dom";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Check, ChevronRight } from "lucide-react";
import { useAuth } from "@/context/AuthContext";
import { supabase } from "@/integrations/supabase/client";

const CheckoutSuccess = () => {
  const { user } = useAuth();
  const navigate = useNavigate();

  useEffect(() => {
    if (!user) {
      // If no user is logged in, redirect to login
      navigate("/auth");
      return;
    }

    // Verify subscription status
    const checkSubscription = async () => {
      try {
        await supabase.functions.invoke('check-subscription');
      } catch (error) {
        console.error("Error checking subscription:", error);
      }
    };

    checkSubscription();
  }, [user, navigate]);

  return (
    <div className="min-h-screen bg-background flex flex-col items-center justify-center p-4">
      <Card className="max-w-md w-full">
        <CardHeader className="text-center">
          <div className="mx-auto bg-green-100 text-green-600 p-3 rounded-full w-16 h-16 flex items-center justify-center mb-4">
            <Check className="h-8 w-8" />
          </div>
          <CardTitle className="text-2xl">Payment Successful!</CardTitle>
          <CardDescription>Thank you for subscribing to RoboMetrics</CardDescription>
        </CardHeader>
        <CardContent className="text-center">
          <p className="mb-4">
            Your subscription has been activated successfully. You now have access to all features included in your plan.
          </p>
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
