
import { useState, useEffect } from "react";
import { DashboardHeader } from "./DashboardHeader";
import { StatCards } from "./StatCards";
import { RobotStatusGrid } from "./RobotStatusGrid";
import { MapView } from "./MapView";
import { robotService } from "@/services/robotService";
import { Robot } from "@/types/robot";
import { useToast } from "@/hooks/use-toast";

export function Dashboard() {
  const [robots, setRobots] = useState<Robot[]>([]);
  const { toast } = useToast();

  useEffect(() => {
    // Initial data load
    const initialData = robotService.getRobots();
    setRobots(initialData);
    
    // Check for warnings
    initialData.forEach(robot => {
      if (robot.status === 'warning' && robot.batteryLevel < 20) {
        toast({
          title: "Low Battery Warning",
          description: `${robot.name} battery level is below 20%`,
          variant: "destructive",
        });
      }
    });
    
    // Set up polling for updates
    const interval = setInterval(() => {
      const updatedRobots = robotService.getRobots();
      setRobots(updatedRobots);
      
      // Check for new warnings
      updatedRobots.forEach(robot => {
        const prevRobot = robots.find(r => r.id === robot.id);
        if (
          robot.status === 'warning' && 
          robot.batteryLevel < 20 && 
          (prevRobot?.batteryLevel || 0) >= 20
        ) {
          toast({
            title: "Low Battery Warning",
            description: `${robot.name} battery level is below 20%`,
            variant: "destructive",
          });
        }
      });
    }, 5000);
    
    return () => clearInterval(interval);
  }, [toast, robots]);

  return (
    <div>
      <DashboardHeader />
      <StatCards robots={robots} />
      <MapView robots={robots} />
      <RobotStatusGrid robots={robots} />
    </div>
  );
}
