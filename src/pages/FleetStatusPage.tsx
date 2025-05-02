
import { MainLayout } from "@/components/layout/MainLayout";
import { FleetStatus } from "@/components/dashboard/FleetStatus";
import { useRobots } from "@/hooks/useRobots";

const FleetStatusPage = () => {
  const { robots, isLoading, error } = useRobots();

  if (isLoading) {
    return (
      <MainLayout>
        <div className="flex items-center justify-center h-96">
          <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-primary"></div>
        </div>
      </MainLayout>
    );
  }

  if (error) {
    return (
      <MainLayout>
        <div className="flex flex-col items-center justify-center h-96 text-center">
          <h2 className="text-xl font-semibold text-destructive mb-2">Error loading robot data</h2>
          <p className="text-muted-foreground">Please try again later</p>
        </div>
      </MainLayout>
    );
  }

  return (
    <MainLayout>
      <div className="space-y-6">
        <div className="flex items-center justify-between">
          <h1 className="text-3xl font-bold tracking-tight">Fleet Status</h1>
        </div>
        <p className="text-muted-foreground">Comprehensive overview of your robot fleet's operational status.</p>
        
        <FleetStatus robots={robots || []} />
      </div>
    </MainLayout>
  );
};

export default FleetStatusPage;
