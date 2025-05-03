
import React, { useEffect } from "react";
import { useNavigate } from "react-router-dom";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Check } from "lucide-react";
import { useToast } from "@/hooks/use-toast";
import { MainLayout } from "@/components/layout/MainLayout";

export default function SubscriptionSuccess() {
  const navigate = useNavigate();
  const { toast } = useToast();
  
  useEffect(() => {
    toast({
      title: "Subscription Successful",
      description: "Your subscription has been activated successfully!",
    });
  }, [toast]);
  
  return (
    <MainLayout>
      <div className="container max-w-4xl mx-auto py-12 px-4">
        <Card className="text-center">
          <CardHeader>
            <div className="w-16 h-16 bg-green-100 rounded-full flex items-center justify-center mx-auto mb-4">
              <Check className="h-8 w-8 text-primary" />
            </div>
            <CardTitle className="text-3xl">Subscription Successful</CardTitle>
          </CardHeader>
          <CardContent>
            <p className="text-muted-foreground mb-6">
              Thank you for subscribing to RoboMetrics! Your account has been upgraded and you now have access to all premium features.
            </p>
            
            <h3 className="text-lg font-semibold mb-2">What's next?</h3>
            <ul className="text-left max-w-md mx-auto space-y-2">
              <li className="flex items-center gap-2">
                <Check className="h-4 w-4 text-primary flex-shrink-0" />
                <span>Explore your new premium features</span>
              </li>
              <li className="flex items-center gap-2">
                <Check className="h-4 w-4 text-primary flex-shrink-0" />
                <span>Configure additional robots for monitoring</span>
              </li>
              <li className="flex items-center gap-2">
                <Check className="h-4 w-4 text-primary flex-shrink-0" />
                <span>Set up custom alerts and notifications</span>
              </li>
              <li className="flex items-center gap-2">
                <Check className="h-4 w-4 text-primary flex-shrink-0" />
                <span>Explore advanced telemetry options</span>
              </li>
            </ul>
          </CardContent>
          <CardFooter className="flex justify-center gap-4">
            <Button onClick={() => navigate("/dashboard")}>
              Go to Dashboard
            </Button>
            <Button variant="outline" onClick={() => navigate("/profile")}>
              Manage Subscription
            </Button>
          </CardFooter>
        </Card>
      </div>
    </MainLayout>
  );
}
