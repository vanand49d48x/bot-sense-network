
import React from "react";
import { Link } from "react-router-dom";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { AlertCircle, ChevronRight } from "lucide-react";

const CheckoutCancel = () => {
  return (
    <div className="min-h-screen bg-background flex flex-col items-center justify-center p-4">
      <Card className="max-w-md w-full">
        <CardHeader className="text-center">
          <div className="mx-auto bg-amber-100 text-amber-600 p-3 rounded-full w-16 h-16 flex items-center justify-center mb-4">
            <AlertCircle className="h-8 w-8" />
          </div>
          <CardTitle className="text-2xl">Payment Cancelled</CardTitle>
          <CardDescription>Your subscription was not completed</CardDescription>
        </CardHeader>
        <CardContent className="text-center">
          <p className="mb-4">
            The payment process was cancelled. No charges have been made to your account.
          </p>
          <p>
            If you experienced any issues during checkout or have questions about our plans, please don't hesitate to contact our support team.
          </p>
        </CardContent>
        <CardFooter className="flex flex-col gap-2">
          <Button asChild className="w-full">
            <Link to="/pricing">
              Try Again
              <ChevronRight className="ml-2 h-4 w-4" />
            </Link>
          </Button>
          <Button asChild variant="outline" className="w-full">
            <Link to="/contact">
              Contact Support
            </Link>
          </Button>
        </CardFooter>
      </Card>
    </div>
  );
};

export default CheckoutCancel;
