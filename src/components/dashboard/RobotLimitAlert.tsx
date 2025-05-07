
import React from "react";
import { Alert, AlertTitle, AlertDescription } from "@/components/ui/alert";
import { AlertCircle } from "lucide-react";
import { Link } from "react-router-dom";
import { Button } from "@/components/ui/button";
import { useSubscriptionLimits } from "@/utils/planRestrictions";

interface RobotLimitAlertProps {
  currentRobotCount: number;
}

export function RobotLimitAlert({ currentRobotCount }: RobotLimitAlertProps) {
  const { limits, planName } = useSubscriptionLimits();
  
  const isAtLimit = currentRobotCount >= limits.robotLimit;
  const isNearLimit = currentRobotCount >= limits.robotLimit - 1;
  
  if (!isAtLimit && !isNearLimit) return null;
  
  return (
    <Alert variant={isAtLimit ? "destructive" : "warning"} className="mb-4">
      <AlertCircle className="h-4 w-4" />
      <AlertTitle>
        {isAtLimit ? "Robot Limit Reached" : "Approaching Robot Limit"}
      </AlertTitle>
      <AlertDescription className="flex flex-col gap-2">
        <p>
          {isAtLimit
            ? `You've reached the maximum of ${limits.robotLimit} robots allowed on your ${planName} plan.`
            : `You're approaching the limit of ${limits.robotLimit} robots on your ${planName} plan.`}
        </p>
        <Button asChild variant="outline" className="mt-2 w-full sm:w-auto">
          <Link to="/pricing">Upgrade Plan</Link>
        </Button>
      </AlertDescription>
    </Alert>
  );
}
