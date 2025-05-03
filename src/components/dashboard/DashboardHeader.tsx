
import { Button } from "@/components/ui/button";
import { RefreshCw } from "lucide-react";
import { Robot } from "@/types/robot";

interface DashboardHeaderProps {
  onRefresh: () => void;
  robots?: Robot[]; // Added robots as an optional prop
}

export function DashboardHeader({ onRefresh, robots }: DashboardHeaderProps) {
  return (
    <div className="flex flex-col md:flex-row justify-between items-start md:items-center mb-6 gap-4">
      <div>
        <h1 className="text-2xl font-bold tracking-tight">Robot Fleet Dashboard</h1>
        <p className="text-muted-foreground mt-1">
          Monitor and manage your robots in real-time.
          {robots && <span className="ml-1">({robots.length} total)</span>}
        </p>
      </div>
      <div className="flex items-center gap-2">
        <Button variant="outline" onClick={onRefresh}>
          <RefreshCw className="h-4 w-4 mr-2" />
          Refresh
        </Button>
      </div>
    </div>
  );
}
