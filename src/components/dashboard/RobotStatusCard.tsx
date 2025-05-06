
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Robot } from "@/types/robot";
import { Progress } from "@/components/ui/progress";
import { Badge } from "@/components/ui/badge";
import { formatDistanceToNow } from "date-fns";
import { AlertDialog, AlertDialogAction, AlertDialogCancel, AlertDialogContent, AlertDialogDescription, AlertDialogFooter, AlertDialogHeader, AlertDialogTitle, AlertDialogTrigger } from "@/components/ui/alert-dialog";
import { ChevronDown, ChevronUp, ClipboardCopy, Trash } from "lucide-react";
import { useRobots } from "@/hooks/useRobots";
import { Button } from "@/components/ui/button";
import { useState } from "react";
import { toast } from "@/components/ui/sonner";

interface RobotStatusCardProps {
  robot: Robot;
  onRobotClick: (robot: Robot) => void;
}

export function RobotStatusCard({ robot, onRobotClick }: RobotStatusCardProps) {
  const { deleteRobot } = useRobots();
  const [isDeleting, setIsDeleting] = useState(false);
  const [showApiInfo, setShowApiInfo] = useState(false);

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

  const handleDelete = async () => {
    try {
      setIsDeleting(true);
      await deleteRobot(robot.id);
      toast("Robot deleted", {
        description: `${robot.name} has been successfully deleted.`
      });
    } catch (error: any) {
      toast.error("Error deleting robot", {
        description: error.message,
      });
    } finally {
      setIsDeleting(false);
    }
  };
  
  const copyRobotId = (e: React.MouseEvent) => {
    // Stop event propagation to prevent opening the details view
    e.stopPropagation();
    
    navigator.clipboard.writeText(robot.id);
    toast("Robot ID copied", {
      description: "The Robot ID has been copied to your clipboard."
    });
  };

  const location = robot.location as { latitude: number, longitude: number } | null;

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
            <AlertDialog>
              <AlertDialogTrigger asChild>
                <Button 
                  variant="ghost" 
                  size="icon" 
                  className="h-8 w-8"
                  onClick={(e) => e.stopPropagation()} // Stop propagation to prevent detail view
                >
                  <Trash className="h-4 w-4 text-muted-foreground hover:text-destructive" />
                  <span className="sr-only">Delete robot</span>
                </Button>
              </AlertDialogTrigger>
              <AlertDialogContent>
                <AlertDialogHeader>
                  <AlertDialogTitle>Are you sure?</AlertDialogTitle>
                  <AlertDialogDescription>
                    This will permanently delete {robot.name} and all associated data. This action cannot be undone.
                  </AlertDialogDescription>
                </AlertDialogHeader>
                <AlertDialogFooter>
                  <AlertDialogCancel>Cancel</AlertDialogCancel>
                  <AlertDialogAction 
                    onClick={handleDelete} 
                    disabled={isDeleting}
                    className="bg-destructive text-destructive-foreground hover:bg-destructive/90"
                  >
                    {isDeleting ? "Deleting..." : "Delete"}
                  </AlertDialogAction>
                </AlertDialogFooter>
              </AlertDialogContent>
            </AlertDialog>
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
              <div className={getTemperatureColor(Number(robot.temperature))}>{robot.temperature}°C</div>
            </div>
            <div>
              <span className="text-muted-foreground">Last Heartbeat</span>
              <div>{getLastHeartbeatText(robot.lastHeartbeat)}</div>
            </div>
          </div>

          <div className="pt-2">
            <Button 
              variant="outline" 
              size="sm" 
              className="w-full flex justify-between items-center text-xs" 
              onClick={(e) => {
                e.stopPropagation(); // Stop propagation to prevent detail view
                setShowApiInfo(!showApiInfo);
              }}
            >
              API Integration
              {showApiInfo ? <ChevronUp size={16} /> : <ChevronDown size={16} />}
            </Button>
            
            {showApiInfo && (
              <div 
                className="mt-2 space-y-3"
                onClick={(e) => e.stopPropagation()} // Stop propagation for the entire API section
              >
                <div className="p-3 bg-muted/50 rounded-md">
                  <div className="text-xs text-muted-foreground mb-1">Robot ID</div>
                  <div className="flex items-center gap-2">
                    <code className="text-xs bg-background p-1 rounded border flex-1 overflow-hidden overflow-ellipsis">
                      {robot.id}
                    </code>
                    <Button 
                      variant="ghost" 
                      size="sm" 
                      className="h-7 w-7 p-0" 
                      onClick={copyRobotId}
                    >
                      <ClipboardCopy size={14} />
                      <span className="sr-only">Copy Robot ID</span>
                    </Button>
                  </div>
                </div>

                <div className="text-xs text-muted-foreground mt-2 px-3">
                  Use the Robot ID and your account API key to send telemetry data via the API.
                </div>
              </div>
            )}
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
