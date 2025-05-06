
import React from "react";
import { MainLayout } from "@/components/layout/MainLayout";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { ArrowRight, Calendar, CreditCard, Loader2, Package } from "lucide-react";
import { Link } from "react-router-dom";
import { useSubscription } from "@/context/SubscriptionContext";
import { useAuth } from "@/context/AuthContext";

const SubscriptionManage = () => {
  const { subscription, isLoading, openCustomerPortal } = useSubscription();
  const { user } = useAuth();
  
  const formatDate = (dateString: string) => {
    return new Date(dateString).toLocaleDateString(undefined, {
      year: "numeric",
      month: "long",
      day: "numeric",
    });
  };
  
  const handleManageSubscription = async () => {
    await openCustomerPortal();
  };
  
  if (isLoading) {
    return (
      <MainLayout>
        <div className="container py-10 max-w-3xl mx-auto">
          <Card>
            <CardContent className="pt-6">
              <div className="flex flex-col items-center justify-center py-8">
                <Loader2 className="h-12 w-12 animate-spin text-primary mb-4" />
                <p className="text-lg">Loading subscription information...</p>
              </div>
            </CardContent>
          </Card>
        </div>
      </MainLayout>
    );
  }
  
  if (!user) {
    return (
      <MainLayout>
        <div className="container py-10 max-w-3xl mx-auto">
          <Card>
            <CardHeader>
              <CardTitle>Subscription Management</CardTitle>
            </CardHeader>
            <CardContent>
              <p className="text-center">
                Please sign in to view and manage your subscription.
              </p>
            </CardContent>
            <CardFooter className="flex justify-center">
              <Button asChild>
                <Link to="/auth">Sign In</Link>
              </Button>
            </CardFooter>
          </Card>
        </div>
      </MainLayout>
    );
  }
  
  return (
    <MainLayout>
      <div className="container py-10 max-w-3xl mx-auto">
        <Card>
          <CardHeader>
            <CardTitle className="text-2xl">Subscription Management</CardTitle>
          </CardHeader>
          <CardContent>
            {subscription && subscription.status === "active" ? (
              <div className="space-y-6">
                <div className="bg-primary/10 p-4 rounded-lg border border-primary/20">
                  <div className="flex items-center gap-3 mb-3">
                    <Package className="h-5 w-5 text-primary" />
                    <h3 className="text-lg font-semibold">Current Plan: {subscription.planName}</h3>
                  </div>
                  <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                    <div className="flex items-center gap-2">
                      <CreditCard className="h-4 w-4 text-muted-foreground" />
                      <span className="text-sm text-muted-foreground">Status: Active</span>
                    </div>
                    {subscription.currentPeriodEnd && (
                      <div className="flex items-center gap-2">
                        <Calendar className="h-4 w-4 text-muted-foreground" />
                        <span className="text-sm text-muted-foreground">
                          Next billing date: {formatDate(subscription.currentPeriodEnd)}
                        </span>
                      </div>
                    )}
                  </div>
                </div>
                
                <div className="space-y-3">
                  <h3 className="text-lg font-semibold">Manage Your Subscription</h3>
                  <p className="text-muted-foreground">
                    You can manage your subscription through our secure customer portal. 
                    From there, you can:
                  </p>
                  <ul className="list-disc pl-5 space-y-1 text-muted-foreground">
                    <li>Update your payment method</li>
                    <li>Change your subscription plan</li>
                    <li>View your billing history</li>
                    <li>Cancel your subscription</li>
                  </ul>
                </div>
                
                <Button onClick={handleManageSubscription} className="w-full">
                  Manage Subscription
                </Button>
              </div>
            ) : (
              <div className="space-y-6">
                <div className="text-center p-6">
                  <Package className="h-12 w-12 text-muted-foreground mx-auto mb-4" />
                  <h3 className="text-lg font-semibold mb-2">No Active Subscription</h3>
                  <p className="text-muted-foreground">
                    You don't currently have an active subscription plan.
                    Explore our subscription options to get access to premium features.
                  </p>
                </div>
                
                <div className="flex justify-center">
                  <Button asChild>
                    <Link to="/pricing">
                      View Plans
                      <ArrowRight className="ml-2 h-4 w-4" />
                    </Link>
                  </Button>
                </div>
              </div>
            )}
          </CardContent>
        </Card>
      </div>
    </MainLayout>
  );
};

export default SubscriptionManage;
