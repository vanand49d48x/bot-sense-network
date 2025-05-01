
import { useParams } from "react-router-dom";
import { MainLayout } from "@/components/layout/MainLayout";
import { RobotTelemetryHistory } from "@/components/dashboard/RobotTelemetryHistory";
import { useRobotDetails } from "@/hooks/useRobotDetails";
import { Button } from "@/components/ui/button";
import { ArrowLeft, Loader2 } from "lucide-react";
import { Link } from "react-router-dom";

const RobotDetails = () => {
  const { robotId } = useParams<{ robotId: string }>();
  const { robot, loading, error } = useRobotDetails(robotId || "");

  if (loading) {
    return (
      <MainLayout>
        <div className="flex items-center justify-center h-[70vh]">
          <div className="flex flex-col items-center gap-2">
            <Loader2 className="h-8 w-8 animate-spin text-primary" />
            <p className="text-muted-foreground">Loading robot data...</p>
          </div>
        </div>
      </MainLayout>
    );
  }

  if (error || !robot) {
    return (
      <MainLayout>
        <div className="flex flex-col items-center justify-center h-[70vh] gap-4">
          <p className="text-destructive">Error loading robot data</p>
          <Button asChild variant="outline">
            <Link to="/dashboard">Return to Dashboard</Link>
          </Button>
        </div>
      </MainLayout>
    );
  }

  return (
    <MainLayout>
      <div className="mb-6">
        <Button variant="ghost" asChild className="mb-2">
          <Link to="/dashboard">
            <ArrowLeft className="mr-1 h-4 w-4" /> Back to Dashboard
          </Link>
        </Button>
        <div className="flex justify-between items-center">
          <h1 className="text-3xl font-bold">
            {robot.name} <span className="text-muted-foreground text-lg ml-2">{robot.model}</span>
          </h1>
        </div>
      </div>

      <RobotTelemetryHistory robotId={robot.id} />
    </MainLayout>
  );
};

export default RobotDetails;
