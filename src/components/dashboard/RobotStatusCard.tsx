
import { Robot } from "@/types/robot";
import { Card, CardContent, CardFooter, CardHeader } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { RobotStatusBadge } from "./RobotStatusBadge";
import { Battery, Thermometer, Eye, Copy, CopyCheck } from "lucide-react";
import { format } from "date-fns";
import { Link } from "react-router-dom";
import { useState } from "react";
import { toast } from "@/components/ui/sonner";

interface RobotStatusCardProps {
  robot: Robot;
}

export function RobotStatusCard({ robot }: RobotStatusCardProps) {
  const [copied, setCopied] = useState(false);
  
  // Format the last heartbeat timestamp
  const formattedLastHeartbeat = robot.lastHeartbeat
    ? format(new Date(robot.lastHeartbeat), "MMM d, h:mm a")
    : "Never";

  // Function to copy robot ID to clipboard
  const copyRobotId = () => {
    navigator.clipboard.writeText(robot.id);
    setCopied(true);
    toast.success("Robot ID copied to clipboard");
    
    // Reset copied state after 2 seconds
    setTimeout(() => {
      setCopied(false);
    }, 2000);
  };

  // Get battery color based on level
  const getBatteryColor = (level: number) => {
    if (level > 50) return "text-green-600";
    if (level > 20) return "text-amber-500";
    return "text-red-600";
  };

  // Get temperature color based on value
  const getTemperatureColor = (temp: number) => {
    if (temp < 30) return "text-blue-500";
    if (temp < 60) return "text-green-600";
    if (temp < 80) return "text-amber-500";
    return "text-red-600";
  };

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
          <div className={`text-sm font-medium ${getBatteryColor(robot.batteryLevel)}`}>
            {robot.batteryLevel}%
          </div>
        </div>
        <div className="flex items-center justify-between gap-2">
          <div className="flex items-center text-sm">
            <Thermometer className="mr-1 h-4 w-4 text-muted-foreground" />
            <span>Temperature</span>
          </div>
          <div className={`text-sm font-medium ${getTemperatureColor(robot.temperature)}`}>
            {robot.temperature}°C
          </div>
        </div>
        <div className="flex items-center justify-between border-t border-dashed pt-2 mt-2">
          <div className="text-xs text-muted-foreground truncate max-w-[70%]" title={robot.id}>
            ID: {robot.id}
          </div>
          <Button
            variant="ghost"
            size="icon"
            className="h-6 w-6"
            onClick={copyRobotId}
            title="Copy robot ID"
          >
            {copied ? (
              <CopyCheck className="h-4 w-4 text-green-500" />
            ) : (
              <Copy className="h-4 w-4" />
            )}
          </Button>
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
