import React, { useEffect } from "react";
import { Alert, AlertTitle, AlertDescription } from "@/components/ui/alert";
import { AlertCircle } from "lucide-react";
import { Link } from "react-router-dom";
import { Button } from "@/components/ui/button";
import { useSubscriptionLimits } from "@/utils/planRestrictions";
import { useToast } from "@/hooks/use-toast";

interface RobotLimitAlertProps {
  currentRobotCount: number;
}

export function RobotLimitAlert({ currentRobotCount }: RobotLimitAlertProps) {
  const { limits, planName, isLoading } = useSubscriptionLimits();
  const { toast } = useToast();
  
  // Only calculate these values when not loading to prevent flashing
  const isAtLimit = !isLoading && currentRobotCount >= limits.robotLimit;
  const isNearLimit = !isLoading && !isAtLimit && currentRobotCount >= limits.robotLimit - 1;
  
  // Log the current limits to help debug
  useEffect(() => {
    console.log(`RobotLimitAlert - Plan: ${planName}, Robot Limit: ${limits.robotLimit}, Current Count: ${currentRobotCount}, Loading: ${isLoading}`);
  }, [limits, planName, currentRobotCount, isLoading]);
  
  // Don't show any alert if we're still loading the subscription data
  // This prevents the alert from showing incorrect information during loading
  if (isLoading) {
    return null;
  }
  
  // Only show the alert if we're at or near the limit
  if (!isAtLimit && !isNearLimit) {
    return null;
  }
  
  // Custom dark mode classes for destructive and warning
  const alertClass = isAtLimit
    ? "bg-destructive/20 border-destructive text-destructive dark:bg-[#2a1515] dark:border-red-700 dark:text-red-300"
    : "bg-amber-500/20 border-amber-500 text-amber-900 dark:bg-amber-900/40 dark:border-amber-700 dark:text-amber-200";

  return (
    <Alert variant="destructive" className={`mb-4 ${alertClass}`}>
      <AlertCircle className="h-4 w-4" />
      <AlertTitle className="font-bold">
        {isAtLimit ? "Robot Limit Reached" : "Approaching Robot Limit"}
      </AlertTitle>
      <AlertDescription className="flex flex-col gap-2">
        <p>
          {isAtLimit
            ? `You've reached the maximum of ${limits.robotLimit} robots allowed on your ${planName} plan.`
            : `You're approaching the limit of ${limits.robotLimit} robots on your ${planName} plan.`}
        </p>
        {isAtLimit && (
          <p className="text-sm">
            Either upgrade your plan or remove existing robots to add new ones.
          </p>
        )}
        <Button
          asChild
          variant="outline"
          className="mt-2 w-full sm:w-auto border-destructive text-destructive dark:border-red-500 dark:text-red-200 dark:hover:bg-red-900/30 dark:hover:text-red-100"
        >
          <Link to="/pricing">Upgrade Plan</Link>
        </Button>
      </AlertDescription>
    </Alert>
  );
}
