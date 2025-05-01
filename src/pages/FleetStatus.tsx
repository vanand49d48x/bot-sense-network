
import { MainLayout } from "@/components/layout/MainLayout";
import { Table, TableBody, TableCell, TableHead, TableHeader, TableRow } from "@/components/ui/table";
import { useRobots } from "@/hooks/useRobots";
import { mapSupabaseRobotToAppRobot } from "@/utils/robotMapper";
import { formatDistanceToNow } from "date-fns";
import { RobotStatusBadge } from "@/components/dashboard/RobotStatusBadge";

export default function FleetStatus() {
  const { robots: supabaseRobots, loading } = useRobots();
  
  // Map Supabase robots to application Robot type
  const robots = supabaseRobots.map(mapSupabaseRobotToAppRobot);
  
  const getBatteryColor = (level: number) => {
    if (level > 50) return "text-robot-online";
    if (level > 20) return "text-robot-warning";
    return "text-robot-offline";
  };

  const getLastHeartbeatText = (lastHeartbeat: string) => {
    try {
      return formatDistanceToNow(new Date(lastHeartbeat), { addSuffix: true });
    } catch (e) {
      return 'Unknown';
    }
  };

  if (loading) {
    return (
      <MainLayout>
        <div className="flex items-center justify-center min-h-[400px]">
          <div className="text-center">
            <div className="animate-pulse-slow">Loading fleet status...</div>
          </div>
        </div>
      </MainLayout>
    );
  }

  return (
    <MainLayout>
      <div className="container py-6">
        <h1 className="text-3xl font-bold tracking-tight mb-6">Fleet Status</h1>
        
        <div className="rounded-md border">
          <Table>
            <TableHeader>
              <TableRow>
                <TableHead>Name</TableHead>
                <TableHead>Status</TableHead>
                <TableHead>Battery</TableHead>
                <TableHead>Temperature</TableHead>
                <TableHead>Last Ping</TableHead>
                <TableHead>Model</TableHead>
                <TableHead>IP Address</TableHead>
              </TableRow>
            </TableHeader>
            <TableBody>
              {robots.map((robot) => (
                <TableRow key={`${robot.id}-${robot.lastHeartbeat}-${robot.batteryLevel}`}>
                  <TableCell className="font-medium">{robot.name}</TableCell>
                  <TableCell>
                    <RobotStatusBadge status={robot.status} />
                  </TableCell>
                  <TableCell className={getBatteryColor(robot.batteryLevel)}>
                    {robot.batteryLevel}%
                  </TableCell>
                  <TableCell>{robot.temperature}°C</TableCell>
                  <TableCell>{getLastHeartbeatText(robot.lastHeartbeat)}</TableCell>
                  <TableCell>{robot.model}</TableCell>
                  <TableCell>{robot.ipAddress}</TableCell>
                </TableRow>
              ))}
            </TableBody>
          </Table>
        </div>

        {robots.length === 0 && (
          <div className="text-center p-12 border border-dashed rounded-lg mt-6">
            <h3 className="text-lg font-medium mb-2">No robots connected yet</h3>
            <p className="text-muted-foreground mb-4">
              Add robots to see your fleet status
            </p>
          </div>
        )}
      </div>
    </MainLayout>
  );
}
