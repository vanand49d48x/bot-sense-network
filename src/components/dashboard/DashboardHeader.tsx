
import { Button } from "@/components/ui/button";

export function DashboardHeader() {
  return (
    <div className="flex flex-col md:flex-row justify-between items-start md:items-center mb-6 gap-4">
      <div>
        <h1 className="text-2xl font-bold tracking-tight">Robot Fleet Dashboard</h1>
        <p className="text-muted-foreground mt-1">
          Monitor and manage your robots in real-time.
        </p>
      </div>
      <div className="flex items-center gap-2">
        <Button variant="outline">Refresh</Button>
        <Button>+ Add Robot</Button>
      </div>
    </div>
  );
}
