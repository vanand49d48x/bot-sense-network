
import { Button } from "@/components/ui/button";
import { RefreshCw } from "lucide-react";
import { useQueryClient } from "@tanstack/react-query";
import { toast } from "sonner";

export function DashboardHeader() {
  const queryClient = useQueryClient();
  
  const handleRefresh = () => {
    queryClient.invalidateQueries({ queryKey: ["robots"] });
    toast("Dashboard refreshed", {
      description: "Robot data has been updated.",
      duration: 3000,
    });
  };

  return (
    <div className="flex flex-col md:flex-row justify-between items-start md:items-center mb-6 gap-4">
      <div>
        <h1 className="text-2xl font-bold tracking-tight">Robot Fleet Dashboard</h1>
        <p className="text-muted-foreground mt-1">
          Monitor and manage your robots in real-time.
        </p>
      </div>
      <div className="flex items-center gap-2">
        <Button variant="outline" onClick={handleRefresh}>
          <RefreshCw className="mr-2 h-4 w-4" />
          Refresh
        </Button>
      </div>
    </div>
  );
}
