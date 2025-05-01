
import { Robot } from "@/types/robot";
import { Card, CardContent, CardFooter, CardHeader } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { RobotStatusBadge } from "./RobotStatusBadge";
import { Battery, Thermometer, Eye } from "lucide-react";
import { format } from "date-fns";
import { Link } from "react-router-dom";

interface RobotStatusCardProps {
  robot: Robot;
}

export function RobotStatusCard({ robot }: RobotStatusCardProps) {
  // Format the last heartbeat timestamp
  const formattedLastHeartbeat = robot.lastHeartbeat
    ? format(new Date(robot.lastHeartbeat), "MMM d, h:mm a")
    : "Never";

  return (
    <Card className="transition-all hover:shadow-md">
      <CardHeader className="pb-2">
        <div className="flex items-center justify-between">
          <div className="font-medium">{robot.name}</div>
          <RobotStatusBadge status={robot.status} />
        </div>
        <div className="text-xs text-muted-foreground">
          {robot.model} · Last seen: {formattedLastHeartbeat}
        </div>
      </CardHeader>
      <CardContent className="space-y-3">
        <div className="flex items-center justify-between gap-2">
          <div className="flex items-center text-sm">
            <Battery className="mr-1 h-4 w-4 text-muted-foreground" />
            <span>Battery</span>
          </div>
          <div className="text-sm font-medium">{robot.batteryLevel}%</div>
        </div>
        <div className="flex items-center justify-between gap-2">
          <div className="flex items-center text-sm">
            <Thermometer className="mr-1 h-4 w-4 text-muted-foreground" />
            <span>Temperature</span>
          </div>
          <div className="text-sm font-medium">{robot.temperature}°C</div>
        </div>
      </CardContent>
      <CardFooter className="pt-0">
        <Button variant="outline" asChild className="w-full">
          <Link to={`/robot/${robot.id}`}>
            <Eye className="mr-2 h-4 w-4" />
            View Telemetry History
          </Link>
        </Button>
      </CardFooter>
    </Card>
  );
}
