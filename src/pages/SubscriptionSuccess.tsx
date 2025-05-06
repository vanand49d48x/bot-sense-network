
import React, { useEffect, useState } from "react";
import { MainLayout } from "@/components/layout/MainLayout";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { CheckCircle, Loader2 } from "lucide-react";
import { Link, useSearchParams, useNavigate } from "react-router-dom";
import { useSubscription } from "@/context/SubscriptionContext";

const SubscriptionSuccess = () => {
  const [searchParams] = useSearchParams();
  const navigate = useNavigate();
  const { subscription, refetchSubscription, isLoading } = useSubscription();
  const [isRefetching, setIsRefetching] = useState(true);
  
  const sessionId = searchParams.get("session_id");
  
  useEffect(() => {
    const fetchSubscriptionData = async () => {
      try {
        setIsRefetching(true);
        await refetchSubscription();
      } catch (error) {
        console.error("Error fetching subscription:", error);
      } finally {
        setIsRefetching(false);
      }
    };
    
    // If we have a session ID, fetch the latest subscription data
    if (sessionId) {
      fetchSubscriptionData();
    } else {
      // If no session ID, redirect to dashboard
      navigate("/dashboard");
    }
  }, [sessionId, refetchSubscription, navigate]);
  
  // Redirect to dashboard if no session ID
  if (!sessionId) {
    return null;
  }
  
  // Show loading while fetching subscription data
  if (isLoading || isRefetching) {
    return (
      <MainLayout>
        <div className="container py-10 max-w-3xl mx-auto">
          <Card className="text-center">
            <CardHeader>
              <CardTitle>Processing Your Subscription</CardTitle>
            </CardHeader>
            <CardContent>
              <div className="flex flex-col items-center justify-center py-8">
                <Loader2 className="h-12 w-12 animate-spin text-primary mb-4" />
                <p className="text-lg">Please wait while we process your subscription...</p>
              </div>
            </CardContent>
          </Card>
        </div>
      </MainLayout>
    );
  }
  
  return (
    <MainLayout>
      <div className="container py-10 max-w-3xl mx-auto">
        <Card className="text-center">
          <CardHeader>
            <CardTitle className="flex items-center justify-center gap-2">
              <CheckCircle className="h-6 w-6 text-green-500" />
              Subscription Successful!
            </CardTitle>
          </CardHeader>
          <CardContent>
            <div className="py-6">
              <h2 className="text-2xl font-bold mb-2">
                Thank you for subscribing to the {subscription?.planName} plan!
              </h2>
              <p className="text-muted-foreground mb-6">
                Your subscription is now active and you have access to all the features included in your plan.
              </p>
              
              <div className="bg-muted p-4 rounded-lg mb-6 text-left">
                <h3 className="font-semibold mb-2">Subscription Details:</h3>
                <ul className="space-y-1">
                  <li><span className="font-medium">Plan:</span> {subscription?.planName}</li>
                  <li>
                    <span className="font-medium">Status:</span> 
                    <span className="ml-1 inline-flex items-center px-2 py-0.5 rounded-full text-xs font-medium bg-green-100 text-green-800">
                      Active
                    </span>
                  </li>
                  {subscription?.currentPeriodEnd && (
                    <li>
                      <span className="font-medium">Next billing date:</span> 
                      {new Date(subscription.currentPeriodEnd).toLocaleDateString()}
                    </li>
                  )}
                </ul>
              </div>
            </div>
          </CardContent>
          <CardFooter className="flex justify-center gap-4">
            <Button asChild variant="default">
              <Link to="/dashboard">Go to Dashboard</Link>
            </Button>
            <Button asChild variant="outline">
              <Link to="/subscription-manage">Manage Subscription</Link>
            </Button>
          </CardFooter>
        </Card>
      </div>
    </MainLayout>
  );
};

export default SubscriptionSuccess;
