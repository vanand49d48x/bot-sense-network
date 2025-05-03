
import { useEffect } from "react";
import { useNavigate, useSearchParams } from "react-router-dom";
import { Card, CardContent, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Check } from "lucide-react";

export default function CheckoutSuccessPage() {
  const navigate = useNavigate();
  const [searchParams] = useSearchParams();
  const plan = searchParams.get("plan") || "subscription";

  // In a real implementation, you would verify the payment was successful here
  useEffect(() => {
    // For demonstration, we'll just log that we should verify the payment
    console.log("Would verify payment success with Stripe here");
  }, []);

  return (
    <div className="min-h-screen flex items-center justify-center bg-background p-4">
      <Card className="w-full max-w-md">
        <CardHeader className="text-center">
          <div className="mx-auto bg-green-100 dark:bg-green-900/20 rounded-full p-3 w-16 h-16 flex items-center justify-center mb-4">
            <Check className="h-8 w-8 text-green-600 dark:text-green-400" />
          </div>
          <CardTitle className="text-2xl">Subscription Successful!</CardTitle>
        </CardHeader>
        <CardContent className="text-center">
          <p className="text-lg mb-2">
            Thank you for subscribing to our {plan.charAt(0).toUpperCase() + plan.slice(1)} plan.
          </p>
          <p className="text-muted-foreground">
            Your subscription is now active. You can start using all features included in your plan immediately.
          </p>
        </CardContent>
        <CardFooter className="flex flex-col space-y-2">
          <Button 
            className="w-full" 
            onClick={() => navigate("/dashboard")}
          >
            Go to Dashboard
          </Button>
          <Button 
            variant="outline"
            className="w-full" 
            onClick={() => navigate("/")}
          >
            Return to Home
          </Button>
        </CardFooter>
      </Card>
    </div>
  );
}
