
import React from "react";
import { Badge } from "@/components/ui/badge";
import { useSubscription } from "@/context/SubscriptionContext";
import { Tooltip, TooltipContent, TooltipProvider, TooltipTrigger } from "@/components/ui/tooltip";
import { Loader2 } from "lucide-react";

interface SubscriptionBadgeProps {
  className?: string;
}

export const SubscriptionBadge: React.FC<SubscriptionBadgeProps> = ({ className }) => {
  const { isSubscribed, plan, currentPeriodEnd, isLoading } = useSubscription();

  if (isLoading) {
    return (
      <Badge variant="outline" className={`${className} flex items-center gap-1 animate-pulse`}>
        <Loader2 className="h-3 w-3 animate-spin" />
        <span>Checking...</span>
      </Badge>
    );
  }

  if (!isSubscribed) {
    return (
      <Badge variant="outline" className={`${className} bg-muted`}>
        Free Plan
      </Badge>
    );
  }

  // Format the date for display
  const formattedDate = currentPeriodEnd
    ? new Intl.DateTimeFormat('en-US', {
        year: 'numeric',
        month: 'short',
        day: 'numeric'
      }).format(currentPeriodEnd)
    : 'Unknown';

  // Determine badge color based on plan
  let badgeVariant: "default" | "secondary" | "destructive" | "outline" = "default";
  
  if (plan === "Starter") {
    badgeVariant = "secondary";
  } else if (plan === "Growth") {
    badgeVariant = "default";
  } else if (plan === "Pro") {
    badgeVariant = "destructive";
  }

  return (
    <TooltipProvider>
      <Tooltip>
        <TooltipTrigger asChild>
          <Badge variant={badgeVariant} className={className}>
            {plan} Plan
          </Badge>
        </TooltipTrigger>
        <TooltipContent>
          <p>Active until {formattedDate}</p>
        </TooltipContent>
      </Tooltip>
    </TooltipProvider>
  );
};
