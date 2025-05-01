
import { Robot } from "@/types/robot";
import { Card, CardContent, CardFooter, CardHeader } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Battery, Thermometer, Eye, Copy, CopyCheck, ChevronDown } from "lucide-react";
import { format, formatDistanceToNow } from "date-fns";
import { Link } from "react-router-dom";
import { useState } from "react";
import { toast } from "@/components/ui/sonner";
import { Progress } from "@/components/ui/progress";
import {
  Collapsible,
  CollapsibleContent,
  CollapsibleTrigger,
} from "@/components/ui/collapsible";

interface RobotStatusCardProps {
  robot: Robot;
}

export function RobotStatusCard({ robot }: RobotStatusCardProps) {
  const [copiedId, setCopiedId] = useState(false);
  const [copiedApiKey, setCopiedApiKey] = useState(false);
  const [isOpen, setIsOpen] = useState(false);
  
  // Format the last heartbeat timestamp as relative time
  const formattedLastHeartbeat = robot.lastHeartbeat
    ? formatDistanceToNow(new Date(robot.lastHeartbeat), { addSuffix: true })
    : "Never";

  // Function to copy robot ID to clipboard
  const copyRobotId = () => {
    navigator.clipboard.writeText(robot.id);
    setCopiedId(true);
    toast.success("Robot ID copied to clipboard");
    
    // Reset copied state after 2 seconds
    setTimeout(() => {
      setCopiedId(false);
    }, 2000);
  };

  // Function to copy API key to clipboard
  const copyApiKey = () => {
    if (!robot.apiKey) return;
    
    navigator.clipboard.writeText(robot.apiKey);
    setCopiedApiKey(true);
    toast.success("API key copied to clipboard");
    
    // Reset copied state after 2 seconds
    setTimeout(() => {
      setCopiedApiKey(false);
    }, 2000);
  };

  // Get battery color based on level
  const getBatteryColor = (level: number) => {
    if (level > 50) return "text-green-600";
    if (level > 20) return "text-amber-500";
    return "text-red-600";
  };

  const getBatteryProgressColor = (level: number) => {
    if (level > 50) return "bg-green-600";
    if (level > 20) return "bg-amber-500";
    return "bg-red-600";
  };

  // Get temperature color based on value
  const getTemperatureColor = (temp: number) => {
    if (temp < 30) return "text-green-600";
    if (temp < 60) return "text-blue-500";
    if (temp < 80) return "text-amber-500";
    return "text-red-600";
  };

  return (
    <Card className="transition-all hover:shadow-md">
      <CardHeader className="pb-0">
        <div className="flex items-center justify-between">
          <div className="text-xl font-bold">{robot.name}</div>
          <Button
            size="sm"
            variant="outline"
            className={`rounded-full px-4 ${
              robot.status === 'online' 
                ? 'bg-red-500 text-white hover:bg-red-600' 
                : robot.status === 'warning' 
                  ? 'bg-amber-500 text-white hover:bg-amber-600' 
                  : 'bg-gray-500 text-white hover:bg-gray-600'
            }`}
          >
            {robot.status === 'online' ? 'Online' : robot.status === 'warning' ? 'Warning' : 'Offline'}
          </Button>
          <Button 
            variant="ghost" 
            size="icon" 
            className="absolute right-1 top-1" 
            onClick={copyRobotId}
          >
            {copiedId ? <CopyCheck size={16} /> : <Copy size={16} />}
          </Button>
        </div>
        <div className="text-gray-500 text-sm">{robot.model}</div>
      </CardHeader>
      <CardContent className="pt-4 space-y-4">
        <div>
          <div className="flex items-center justify-between mb-1">
            <div className="font-medium">Battery</div>
            <div className={`font-bold text-lg ${getBatteryColor(robot.batteryLevel)}`}>
              {robot.batteryLevel}%
            </div>
          </div>
          <Progress 
            value={robot.batteryLevel} 
            className="h-2" 
            indicatorClassName={getBatteryProgressColor(robot.batteryLevel)}
          />
        </div>
        
        <div className="grid grid-cols-2 gap-4">
          <div>
            <div className="text-gray-500 text-sm">Temperature</div>
            <div className={`font-medium ${getTemperatureColor(robot.temperature)}`}>
              {robot.temperature}°C
            </div>
          </div>
          <div>
            <div className="text-gray-500 text-sm">Last Heartbeat</div>
            <div className="font-medium">
              {formattedLastHeartbeat}
            </div>
          </div>
        </div>

        <Collapsible open={isOpen} onOpenChange={setIsOpen} className="w-full">
          <CollapsibleTrigger asChild>
            <Button variant="ghost" className="flex w-full justify-between p-0 h-auto">
              <span className="font-medium">API Integration</span>
              <ChevronDown className={`h-4 w-4 transition-transform ${isOpen ? "transform rotate-180" : ""}`} />
            </Button>
          </CollapsibleTrigger>
          <CollapsibleContent className="pt-2">
            <div className="rounded-md bg-muted p-3">
              <div className="text-xs text-muted-foreground mb-1">Robot ID</div>
              <div className="flex items-center justify-between">
                <code className="text-xs bg-muted rounded px-1 py-0.5 font-mono truncate max-w-[80%]">
                  {robot.id}
                </code>
                <Button
                  variant="ghost"
                  size="sm"
                  className="h-6 w-6 p-0"
                  onClick={copyRobotId}
                >
                  {copiedId ? (
                    <CopyCheck className="h-3 w-3 text-green-500" />
                  ) : (
                    <Copy className="h-3 w-3" />
                  )}
                </Button>
              </div>
              {robot.apiKey && (
                <>
                  <div className="text-xs text-muted-foreground mt-2 mb-1">API Key</div>
                  <div className="flex items-center justify-between">
                    <code className="text-xs font-mono bg-muted rounded truncate max-w-[80%]">
                      {robot.apiKey}
                    </code>
                    <Button
                      variant="ghost"
                      size="sm"
                      className="h-6 w-6 p-0"
                      onClick={copyApiKey}
                    >
                      {copiedApiKey ? (
                        <CopyCheck className="h-3 w-3 text-green-500" />
                      ) : (
                        <Copy className="h-3 w-3" />
                      )}
                    </Button>
                  </div>
                </>
              )}
            </div>
          </CollapsibleContent>
        </Collapsible>
      </CardContent>
      <CardFooter className="pt-2">
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
