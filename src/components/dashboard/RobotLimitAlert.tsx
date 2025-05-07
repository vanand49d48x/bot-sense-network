
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
  const { limits, planName } = useSubscriptionLimits();
  const { toast } = useToast();
  
  const isAtLimit = currentRobotCount >= limits.robotLimit;
  const isNearLimit = currentRobotCount >= limits.robotLimit - 1;
  
  // Log the current limits to help debug
  useEffect(() => {
    console.log(`RobotLimitAlert - Plan: ${planName}, Robot Limit: ${limits.robotLimit}, Current Count: ${currentRobotCount}`);
  }, [limits, planName, currentRobotCount]);
  
  if (!isAtLimit && !isNearLimit) return null;
  
  return (
    <Alert variant="destructive" className={`mb-4 ${isAtLimit ? "bg-destructive/20" : "bg-amber-500/20"}`}>
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
