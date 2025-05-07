
import { Alert, AlertDescription, AlertTitle } from "@/components/ui/alert";
import { LucideIcon } from "lucide-react";
import { Button } from "@/components/ui/button";
import { Link } from "react-router-dom";

interface PlanFeatureAlertProps {
  title: string;
  description: string;
  icon: LucideIcon;
  showUpgradeButton?: boolean;
}

export function PlanFeatureAlert({ 
  title, 
  description, 
  icon: Icon,
  showUpgradeButton = true 
}: PlanFeatureAlertProps) {
  return (
    <Alert className="mt-4">
      <Icon className="h-4 w-4" />
      <AlertTitle>{title}</AlertTitle>
      <AlertDescription className="flex flex-col">
        <p>{description}</p>
        {showUpgradeButton && (
          <Button asChild variant="outline" size="sm" className="mt-2 w-full sm:w-auto">
            <Link to="/pricing">Upgrade Plan</Link>
          </Button>
        )}
      </AlertDescription>
    </Alert>
  );
}
