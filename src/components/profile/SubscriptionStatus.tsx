
import React, { useEffect, useState } from "react";
import { Link } from "react-router-dom";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { supabase } from "@/integrations/supabase/client";
import { useToast } from "@/hooks/use-toast";
import { CheckCircle, AlertTriangle, Clock, Shield, RefreshCcw, Loader2 } from "lucide-react";

interface SubscriptionInfo {
  active: boolean;
  plan: string | null;
  subscription_end: string | null;
  subscription_id?: string;
}

const SubscriptionStatus = () => {
  const [subscription, setSubscription] = useState<SubscriptionInfo | null>(null);
  const [loading, setLoading] = useState(true);
  const [portalLoading, setPortalLoading] = useState(false);
  const { toast } = useToast();

  const checkSubscription = async () => {
    try {
      setLoading(true);
      
      console.log("Checking subscription status...");
      const { data, error } = await supabase.functions.invoke('check-subscription');
      
      if (error) {
        throw error;
      }
      
      console.log("Subscription data:", data);
      setSubscription(data);
      
      if (data.active) {
        toast({
          title: "Subscription Active",
          description: `Your ${data.plan} plan is active`,
        });
      }
    } catch (error: any) {
      console.error("Error checking subscription:", error);
      toast({
        title: "Error",
        description: "Failed to check subscription status. Please try again.",
        variant: "destructive",
      });
    } finally {
      setLoading(false);
    }
  };

  const openCustomerPortal = async () => {
    try {
      setPortalLoading(true);
      toast({
        title: "Opening Portal",
        description: "Preparing subscription management portal...",
      });
      
      const { data, error } = await supabase.functions.invoke('customer-portal');
      
      if (error) {
        throw error;
      }
      
      window.location.href = data.url;
    } catch (error: any) {
      console.error("Error opening customer portal:", error);
      toast({
        title: "Error",
        description: "Failed to open subscription management portal. Please try again.",
        variant: "destructive",
      });
    } finally {
      setPortalLoading(false);
    }
  };

  useEffect(() => {
    checkSubscription();
    
    // Auto-refresh subscription status every 15 seconds
    const interval = setInterval(checkSubscription, 15000);
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

  return (
    <Card className="h-full">
      <CardHeader>
        <div className="flex items-center justify-between">
          <CardTitle className="flex items-center gap-2">
            {loading ? (
              <>
                <Loader2 className="h-5 w-5 animate-spin text-muted-foreground" />
                <span>Checking Subscription...</span>
              </>
            ) : subscription?.active ? (
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
          <Button 
            variant="ghost" 
            size="sm"
            onClick={checkSubscription}
            disabled={loading}
          >
            <RefreshCcw className={`h-4 w-4 ${loading ? 'animate-spin' : ''}`} />
          </Button>
        </div>
        <CardDescription>
          {loading ? "Loading subscription details..." : 
            subscription?.active 
              ? `You are currently on the ${subscription.plan} plan`
              : "You are currently on the Free plan"}
        </CardDescription>
      </CardHeader>
      
      <CardContent>
        {loading ? (
          <div className="flex items-center justify-center py-6">
            <Loader2 className="h-8 w-8 animate-spin text-primary/70" />
          </div>
        ) : subscription?.active ? (
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
            
            <div className="bg-muted/50 p-3 rounded-md mt-4">
              <h4 className="text-sm font-medium mb-2">Your Plan Benefits</h4>
              <ul className="space-y-1">
                {subscription.plan === "Starter" && (
                  <>
                    <li className="text-xs flex items-center gap-1"><CheckCircle className="h-3 w-3 text-green-500" />Monitor up to 5 robots</li>
                    <li className="text-xs flex items-center gap-1"><CheckCircle className="h-3 w-3 text-green-500" />30 days of telemetry history</li>
                    <li className="text-xs flex items-center gap-1"><CheckCircle className="h-3 w-3 text-green-500" />SMS & Email alerts</li>
                  </>
                )}
                {subscription.plan === "Growth" && (
                  <>
                    <li className="text-xs flex items-center gap-1"><CheckCircle className="h-3 w-3 text-green-500" />Monitor up to 20 robots</li>
                    <li className="text-xs flex items-center gap-1"><CheckCircle className="h-3 w-3 text-green-500" />90 days of telemetry history</li>
                    <li className="text-xs flex items-center gap-1"><CheckCircle className="h-3 w-3 text-green-500" />Priority support</li>
                  </>
                )}
                {subscription.plan === "Pro" && (
                  <>
                    <li className="text-xs flex items-center gap-1"><CheckCircle className="h-3 w-3 text-green-500" />Monitor up to 100 robots</li>
                    <li className="text-xs flex items-center gap-1"><CheckCircle className="h-3 w-3 text-green-500" />1 year of telemetry history</li>
                    <li className="text-xs flex items-center gap-1"><CheckCircle className="h-3 w-3 text-green-500" />24/7 support & Advanced security</li>
                  </>
                )}
              </ul>
            </div>
          </div>
        ) : (
          <div className="p-4 bg-muted/50 rounded-md">
            <p className="text-sm mb-3">
              Upgrade to a paid plan to access premium features like:
            </p>
            <ul className="space-y-2 text-sm">
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
            {portalLoading ? (
              <>
                <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                Opening Portal...
              </>
            ) : (
              "Manage Subscription"
            )}
          </Button>
        ) : (
          <Button 
            className="w-full" 
            asChild
          >
            <Link to="/pricing">Upgrade Now</Link>
          </Button>
        )}
      </CardFooter>
    </Card>
  );
};

export default SubscriptionStatus;
