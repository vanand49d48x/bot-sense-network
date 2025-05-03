
import React, { useEffect } from "react";
import { useQuery } from "@tanstack/react-query";
import { DashboardHeader } from "./DashboardHeader";
import { StatCards } from "./StatCards";
import { RobotStatusGrid } from "./RobotStatusGrid";
import { useAuth } from "@/context/AuthContext";
import { useToast } from "@/components/ui/use-toast";
import { useLocation } from "react-router-dom";
import { useSubscription } from "@/context/SubscriptionContext";
import { Alert, AlertDescription, AlertTitle } from "@/components/ui/alert";
import { CheckCircle, AlertTriangle } from "lucide-react";
import { fetchRobots } from "@/services/robotService";

export function Dashboard() {
  const { user } = useAuth();
  const { toast } = useToast();
  const location = useLocation();
  const { isSubscribed, plan, checkSubscription } = useSubscription();
  
  // Fetch robots data
  const { data: robots = [], isLoading, isError, refetch } = useQuery({
    queryKey: ["robots", user?.id],
    queryFn: fetchRobots,
    enabled: !!user,
  });
  
  // Handle refresh action
  const handleRefresh = () => {
    refetch();
  };
  
  // Check for payment status in URL params on mount
  useEffect(() => {
    const params = new URLSearchParams(location.search);
    const paymentStatus = params.get('payment');
    
    if (paymentStatus === 'success') {
      toast({
        title: "Payment successful!",
        description: "Your subscription has been activated.",
      });
      // Remove the query param from the URL without refreshing the page
      const newUrl = window.location.pathname;
      window.history.replaceState({}, document.title, newUrl);
      
      // Check subscription status after a successful payment
      checkSubscription();
    }
  }, [location, toast, checkSubscription]);

  return (
    <div className="space-y-6">
      <DashboardHeader onRefresh={handleRefresh} robots={robots} />
      
      {location.search.includes('payment=success') && (
        <Alert className="bg-green-50 dark:bg-green-900/20 border-green-200 dark:border-green-900">
          <CheckCircle className="h-4 w-4 text-green-600 dark:text-green-400" />
          <AlertTitle>Payment Successful!</AlertTitle>
          <AlertDescription>
            Your subscription has been activated. You now have access to {plan} plan features.
          </AlertDescription>
        </Alert>
      )}
      
      {location.search.includes('payment=canceled') && (
        <Alert className="bg-amber-50 dark:bg-amber-900/20 border-amber-200 dark:border-amber-900">
          <AlertTriangle className="h-4 w-4 text-amber-600 dark:text-amber-400" />
          <AlertTitle>Payment Canceled</AlertTitle>
          <AlertDescription>
            Your subscription payment was canceled. You can try again from the pricing page.
          </AlertDescription>
        </Alert>
      )}
      
      <StatCards robots={robots} />
      <RobotStatusGrid robots={robots} />
    </div>
  );
}
