
import { RobotStatusCard } from "./RobotStatusCard";
import { Robot } from "@/types/robot";
import { Table, TableBody, TableCell, TableHead, TableHeader, TableRow } from "@/components/ui/table";
import { useState } from "react";
import { Button } from "@/components/ui/button";
import { TableIcon, Grid3X3Icon } from "lucide-react";
import { formatDistanceToNow } from "date-fns";
import { Link } from "react-router-dom";
import { RobotStatusBadge } from "./RobotStatusBadge";

interface RobotStatusGridProps {
  robots: Robot[];
}

export function RobotStatusGrid({ robots }: RobotStatusGridProps) {
  const [viewMode, setViewMode] = useState<'cards' | 'table'>('cards');

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

  return (
    <div className="mt-6">
      <div className="flex justify-between items-center mb-4">
        <h2 className="text-lg font-semibold">Robot Status</h2>
        <div className="flex items-center gap-2">
          <div className="rounded-md border border-input flex">
            <Button 
              variant={viewMode === 'cards' ? "secondary" : "ghost"} 
              size="sm"
              onClick={() => setViewMode('cards')}
              className="rounded-r-none"
            >
              <Grid3X3Icon className="h-4 w-4 mr-1" />
              Cards
            </Button>
            <Button 
              variant={viewMode === 'table' ? "secondary" : "ghost"} 
              size="sm"
              onClick={() => setViewMode('table')}
              className="rounded-l-none"
            >
              <TableIcon className="h-4 w-4 mr-1" />
              Table
            </Button>
          </div>
          <Button variant="outline" size="sm" asChild>
            <Link to="/integration">Integration Guide</Link>
          </Button>
        </div>
      </div>

      {viewMode === 'cards' ? (
        <div className="grid gap-6 md:grid-cols-2 lg:grid-cols-3">
          {robots.map((robot) => (
            <RobotStatusCard key={`${robot.id}-${robot.lastHeartbeat}`} robot={robot} />
          ))}
        </div>
      ) : (
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
                </TableRow>
              ))}
            </TableBody>
          </Table>
        </div>
      )}
      
      {robots.length === 0 && (
        <div className="text-center p-12 border border-dashed rounded-lg">
          <h3 className="text-lg font-medium mb-2">No robots connected yet</h3>
          <p className="text-muted-foreground mb-4">
            Use the Integration Guide to connect your first robot
          </p>
          <Button asChild>
            <Link to="/integration">View Integration Guide</Link>
          </Button>
        </div>
      )}
    </div>
  );
}
