
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Robot } from "@/types/robot";
import { Progress } from "@/components/ui/progress";
import { Badge } from "@/components/ui/badge";
import { formatDistanceToNow } from "date-fns";

interface RobotStatusCardProps {
  robot: Robot;
}

export function RobotStatusCard({ robot }: RobotStatusCardProps) {
  const getStatusColor = (status: string) => {
    switch (status) {
      case 'online':
        return 'bg-robot-online';
      case 'offline':
        return 'bg-robot-offline';
      case 'warning':
        return 'bg-robot-warning';
      default:
        return 'bg-muted';
    }
  };

  const getBatteryColor = (level: number) => {
    if (level > 50) return 'text-robot-online';
    if (level > 20) return 'text-robot-warning';
    return 'text-robot-offline';
  };

  const getTemperatureColor = (temp: number) => {
    if (temp < 35) return 'text-robot-online';
    if (temp < 40) return 'text-robot-warning';
    return 'text-robot-offline';
  };

  const getLastHeartbeatText = (lastHeartbeat: string) => {
    try {
      return formatDistanceToNow(new Date(lastHeartbeat), { addSuffix: true });
    } catch (e) {
      return 'Unknown';
    }
  };

  return (
    <Card className="animate-fade-in hover:shadow-md transition-shadow">
      <CardHeader className="pb-2">
        <div className="flex justify-between items-start">
          <div>
            <CardTitle className="text-lg font-bold">{robot.name}</CardTitle>
            <div className="text-sm text-muted-foreground">{robot.model}</div>
          </div>
          <div className="flex items-center gap-2">
            <div className="relative">
              <Badge variant={robot.status === 'online' ? 'default' : robot.status === 'warning' ? 'outline' : 'destructive'}>
                {robot.status === 'online' && (
                  <span className="absolute top-1/2 -left-1 -translate-y-1/2 w-2 h-2 rounded-full bg-robot-online animate-ping-slow"></span>
                )}
                <span className="ml-2">{robot.status === 'online' ? 'Online' : robot.status === 'warning' ? 'Warning' : 'Offline'}</span>
              </Badge>
            </div>
          </div>
        </div>
      </CardHeader>
      <CardContent>
        <div className="space-y-3">
          <div>
            <div className="flex justify-between text-sm mb-1">
              <span>Battery</span>
              <span className={getBatteryColor(robot.batteryLevel)}>{robot.batteryLevel}%</span>
            </div>
            <Progress value={robot.batteryLevel} className="h-1.5" />
          </div>
          
          <div className="grid grid-cols-2 gap-4 text-sm">
            <div>
              <span className="text-muted-foreground">Temperature</span>
              <div className={getTemperatureColor(robot.temperature)}>{robot.temperature}°C</div>
            </div>
            <div>
              <span className="text-muted-foreground">Last Heartbeat</span>
              <div>{getLastHeartbeatText(robot.lastHeartbeat)}</div>
            </div>
            <div>
              <span className="text-muted-foreground">IP Address</span>
              <div>{robot.ipAddress}</div>
            </div>
            <div>
              <span className="text-muted-foreground">Error Count</span>
              <div>{robot.errorCount}</div>
            </div>
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
