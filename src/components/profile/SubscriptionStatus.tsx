
import React, { useEffect, useState } from "react";
import { Link } from "react-router-dom";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { supabase } from "@/integrations/supabase/client";
import { useToast } from "@/hooks/use-toast";
import { CheckCircle, AlertTriangle, Clock, Shield } from "lucide-react";

interface SubscriptionInfo {
  active: boolean;
  plan: string | null;
  subscription_end: string | null;
}

const SubscriptionStatus = () => {
  const [subscription, setSubscription] = useState<SubscriptionInfo | null>(null);
  const [loading, setLoading] = useState(true);
  const [portalLoading, setPortalLoading] = useState(false);
  const { toast } = useToast();

  const checkSubscription = async () => {
    try {
      setLoading(true);
      const { data, error } = await supabase.functions.invoke('check-subscription');
      if (error) throw error;
      setSubscription(data);
    } catch (error: any) {
      console.error("Error checking subscription:", error);
      toast({
        title: "Error",
        description: "Failed to check subscription status",
        variant: "destructive",
      });
    } finally {
      setLoading(false);
    }
  };

  const openCustomerPortal = async () => {
    try {
      setPortalLoading(true);
      const { data, error } = await supabase.functions.invoke('customer-portal');
      if (error) throw error;
      window.location.href = data.url;
    } catch (error: any) {
      console.error("Error opening customer portal:", error);
      toast({
        title: "Error",
        description: "Failed to open subscription management portal",
        variant: "destructive",
      });
    } finally {
      setPortalLoading(false);
    }
  };

  useEffect(() => {
    checkSubscription();
    
    // Refresh subscription status every 10 seconds
    const interval = setInterval(checkSubscription, 10000);
    return () => clearInterval(interval);
  }, []);

  const formatDate = (dateStr: string | null) => {
    if (!dateStr) return "N/A";
    return new Date(dateStr).toLocaleDateString(undefined, {
      year: 'numeric',
      month: 'long',
      day: 'numeric'
    });
  };

  if (loading) {
    return (
      <Card>
        <CardHeader>
          <CardTitle>Subscription</CardTitle>
          <CardDescription>Loading subscription information...</CardDescription>
        </CardHeader>
      </Card>
    );
  }

  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center gap-2">
          {subscription?.active ? (
            <>
              <CheckCircle className="h-5 w-5 text-green-500" />
              <span>Active Subscription</span>
            </>
          ) : (
            <>
              <AlertTriangle className="h-5 w-5 text-amber-500" />
              <span>No Active Subscription</span>
            </>
          )}
        </CardTitle>
        <CardDescription>
          {subscription?.active 
            ? `You are currently on the ${subscription.plan} plan`
            : "You are currently on the Free plan"}
        </CardDescription>
      </CardHeader>
      
      <CardContent>
        {subscription?.active && (
          <div className="space-y-4">
            <div className="flex items-center gap-2">
              <Clock className="h-4 w-4 text-muted-foreground" />
              <span className="text-sm">
                Next billing date: {formatDate(subscription.subscription_end)}
              </span>
            </div>
            
            <div className="flex items-center gap-2">
              <Shield className="h-4 w-4 text-muted-foreground" />
              <span className="text-sm">
                Manage your subscription details securely via Stripe
              </span>
            </div>
          </div>
        )}

        {!subscription?.active && (
          <div className="p-4 bg-muted/50 rounded-md">
            <p className="text-sm">
              Upgrade to a paid plan to access premium features like:
            </p>
            <ul className="mt-2 space-y-1 text-sm">
              <li className="flex items-center gap-1">
                <CheckCircle className="h-3 w-3 text-green-500" />
                <span>Monitor more robots</span>
              </li>
              <li className="flex items-center gap-1">
                <CheckCircle className="h-3 w-3 text-green-500" />
                <span>Extended telemetry history</span>
              </li>
              <li className="flex items-center gap-1">
                <CheckCircle className="h-3 w-3 text-green-500" />
                <span>Advanced alerts and notifications</span>
              </li>
            </ul>
          </div>
        )}
      </CardContent>
      
      <CardFooter className="flex flex-col gap-2">
        {subscription?.active ? (
          <Button 
            variant="outline" 
            className="w-full" 
            onClick={openCustomerPortal}
            disabled={portalLoading}
          >
            {portalLoading ? "Opening..." : "Manage Subscription"}
          </Button>
        ) : (
          <Button 
            className="w-full" 
            asChild
          >
            <Link to="/pricing">Upgrade</Link>
          </Button>
        )}
        
        <Button 
          variant="ghost" 
          className="w-full text-sm" 
          onClick={checkSubscription}
          disabled={loading}
        >
          Refresh Status
        </Button>
      </CardFooter>
    </Card>
  );
};

export default SubscriptionStatus;
