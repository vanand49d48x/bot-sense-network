
import React from "react";
import { useSubscription } from "@/context/SubscriptionContext";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Separator } from "@/components/ui/separator";
import { useNavigate } from "react-router-dom";
import { CheckCircle, AlertCircle, RotateCw } from "lucide-react";
import { SubscriptionBadge } from "./SubscriptionBadge";

export function SubscriptionManagement() {
  const { isSubscribed, plan, currentPeriodEnd, isLoading, checkSubscription, openCustomerPortal } = useSubscription();
  const navigate = useNavigate();
  
  const formatDate = (date: Date | null) => {
    if (!date) return "N/A";
    return new Intl.DateTimeFormat('en-US', {
      year: 'numeric',
      month: 'long',
      day: 'numeric'
    }).format(date);
  };

  return (
    <Card className="mb-8">
      <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
        <div>
          <CardTitle className="text-xl font-semibold">Subscription</CardTitle>
          <CardDescription>Manage your subscription plan</CardDescription>
        </div>
        <Button variant="outline" size="sm" onClick={checkSubscription} disabled={isLoading}>
          <RotateCw className="h-4 w-4 mr-2" />
          Refresh
        </Button>
      </CardHeader>
      <CardContent>
        {isLoading ? (
          <div className="flex items-center justify-center p-6">
            <div className="flex flex-col items-center gap-2">
              <RotateCw className="h-8 w-8 animate-spin text-primary" />
              <p className="text-sm text-muted-foreground">Checking subscription status...</p>
            </div>
          </div>
        ) : (
          <>
            <div className="flex items-center gap-4 mb-4">
              <div className="bg-muted/60 p-3 rounded-full">
                {isSubscribed ? (
                  <CheckCircle className="h-6 w-6 text-green-500" />
                ) : (
                  <AlertCircle className="h-6 w-6 text-yellow-500" />
                )}
              </div>
              <div>
                <h3 className="text-lg font-medium flex items-center gap-2">
                  Current Plan: 
                  <SubscriptionBadge />
                </h3>
                {isSubscribed ? (
                  <p className="text-sm text-muted-foreground">
                    Your subscription is active until {formatDate(currentPeriodEnd)}
                  </p>
                ) : (
                  <p className="text-sm text-muted-foreground">
                    You are currently on the Free plan with limited features
                  </p>
                )}
              </div>
            </div>

            <Separator className="my-4" />
            
            <div className="space-y-4">
              <div>
                <h4 className="font-medium mb-2">Subscription Benefits</h4>
                <ul className="space-y-2 text-sm list-disc pl-5">
                  {isSubscribed ? (
                    <>
                      {plan === "Starter" && (
                        <>
                          <li>Up to 5 robots</li>
                          <li>7 days telemetry history</li>
                          <li>Email alerts</li>
                          <li>ROS/Arduino SDK support</li>
                        </>
                      )}
                      {plan === "Growth" && (
                        <>
                          <li>Up to 25 robots</li>
                          <li>30 days telemetry history</li>
                          <li>Custom metrics</li>
                          <li>Real-time alerts</li>
                          <li>Location map</li>
                        </>
                      )}
                      {plan === "Pro" && (
                        <>
                          <li>100+ robots</li>
                          <li>90 days telemetry history</li>
                          <li>SLA + Priority support</li>
                          <li>Custom alerting rules</li>
                          <li>Export + team access</li>
                        </>
                      )}
                    </>
                  ) : (
                    <>
                      <li>1 robot</li>
                      <li>3 days telemetry history</li>
                      <li>Basic API access</li>
                      <li>Live dashboard</li>
                    </>
                  )}
                </ul>
              </div>
            </div>
          </>
        )}
      </CardContent>
      <CardFooter className="flex flex-col sm:flex-row gap-3">
        {isSubscribed ? (
          <Button className="w-full sm:w-auto" onClick={openCustomerPortal}>
            Manage Subscription
          </Button>
        ) : (
          <Button className="w-full sm:w-auto" onClick={() => navigate("/pricing")}>
            Upgrade Plan
          </Button>
        )}
        <Button variant="outline" className="w-full sm:w-auto" onClick={() => navigate("/pricing")}>
          View Plans
        </Button>
      </CardFooter>
    </Card>
  );
}
