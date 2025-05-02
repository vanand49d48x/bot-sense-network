
import { MainLayout } from "@/components/layout/MainLayout";
import { useRobots } from "@/hooks/useRobots";
import { Robot } from "@/types/robot";
import { mapSupabaseRobotToAppRobot } from "@/utils/robotMapper";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { AlertTriangle, Bell, BellRing, Battery, Thermometer } from "lucide-react";
import { useEffect, useState } from "react";
import { RobotStatusBadge } from "@/components/dashboard/RobotStatusBadge";

const Alerts = () => {
  const { robots: supabaseRobots, loading } = useRobots();
  const [robots, setRobots] = useState<Robot[]>([]);
  
  // Map Supabase robots to application Robot type
  useEffect(() => {
    setRobots(supabaseRobots.map(mapSupabaseRobotToAppRobot));
  }, [supabaseRobots]);

  // Filter robots with warnings or offline status
  const robotsWithAlerts = robots.filter(
    robot => robot.status === 'warning' || robot.status === 'offline' || robot.batteryLevel < 20 || robot.errorCount > 0
  );
  
  // Group alerts by type
  const lowBatteryRobots = robots.filter(robot => robot.batteryLevel < 20);
  const offlineRobots = robots.filter(robot => robot.status === 'offline');
  const errorRobots = robots.filter(robot => robot.errorCount > 0);
  const highTempRobots = robots.filter(robot => robot.temperature > 35);

  if (loading) {
    return (
      <MainLayout>
        <div className="flex items-center justify-center min-h-[400px]">
          <div className="text-center">
            <div className="animate-pulse-slow">Loading alerts data...</div>
          </div>
        </div>
      </MainLayout>
    );
  }

  return (
    <MainLayout>
      <div className="space-y-6">
        <div className="flex items-center justify-between">
          <div>
            <h1 className="text-3xl font-bold tracking-tight">Robot Alerts</h1>
            <p className="text-muted-foreground mt-1">
              Monitor and respond to critical issues with your robot fleet
            </p>
          </div>
        </div>
        
        <div className="grid grid-cols-1 md:grid-cols-2 xl:grid-cols-4 gap-4">
          <Card className={`border-l-4 ${lowBatteryRobots.length > 0 ? 'border-l-yellow-500' : 'border-l-green-500'}`}>
            <CardHeader className="pb-2">
              <CardTitle className="text-base flex items-center gap-2">
                <Battery className="h-5 w-5" />
                <span>Low Battery</span>
              </CardTitle>
            </CardHeader>
            <CardContent>
              <div className="text-2xl font-bold">{lowBatteryRobots.length}</div>
              <p className="text-xs text-muted-foreground">Robots with battery under 20%</p>
            </CardContent>
          </Card>
          
          <Card className={`border-l-4 ${offlineRobots.length > 0 ? 'border-l-red-500' : 'border-l-green-500'}`}>
            <CardHeader className="pb-2">
              <CardTitle className="text-base flex items-center gap-2">
                <BellRing className="h-5 w-5" />
                <span>Offline</span>
              </CardTitle>
            </CardHeader>
            <CardContent>
              <div className="text-2xl font-bold">{offlineRobots.length}</div>
              <p className="text-xs text-muted-foreground">Robots currently offline</p>
            </CardContent>
          </Card>
          
          <Card className={`border-l-4 ${errorRobots.length > 0 ? 'border-l-red-500' : 'border-l-green-500'}`}>
            <CardHeader className="pb-2">
              <CardTitle className="text-base flex items-center gap-2">
                <AlertTriangle className="h-5 w-5" />
                <span>Errors</span>
              </CardTitle>
            </CardHeader>
            <CardContent>
              <div className="text-2xl font-bold">{errorRobots.length}</div>
              <p className="text-xs text-muted-foreground">Robots reporting errors</p>
            </CardContent>
          </Card>
          
          <Card className={`border-l-4 ${highTempRobots.length > 0 ? 'border-l-yellow-500' : 'border-l-green-500'}`}>
            <CardHeader className="pb-2">
              <CardTitle className="text-base flex items-center gap-2">
                <Thermometer className="h-5 w-5" />
                <span>High Temperature</span>
              </CardTitle>
            </CardHeader>
            <CardContent>
              <div className="text-2xl font-bold">{highTempRobots.length}</div>
              <p className="text-xs text-muted-foreground">Robots with high temperature</p>
            </CardContent>
          </Card>
        </div>
        
        {robotsWithAlerts.length > 0 ? (
          <Card>
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                <AlertTriangle className="h-5 w-5" />
                <span>Active Alerts</span>
              </CardTitle>
              <CardDescription>
                Active issues requiring attention ({robotsWithAlerts.length})
              </CardDescription>
            </CardHeader>
            <CardContent>
              <div className="space-y-4">
                {robotsWithAlerts.map(robot => (
                  <Card key={robot.id} className="bg-muted/40">
                    <CardContent className="p-4">
                      <div className="flex flex-col md:flex-row md:items-center justify-between gap-2">
                        <div className="flex items-center gap-2">
                          <RobotStatusBadge status={robot.status} />
                          <span className="font-medium">{robot.name}</span>
                        </div>
                        <div className="flex flex-wrap gap-2 text-sm">
                          {robot.status === 'offline' && (
                            <span className="px-2 py-1 bg-red-100 dark:bg-red-900/30 text-red-700 dark:text-red-400 rounded-md flex items-center gap-1">
                              <BellRing className="h-3 w-3" /> Offline
                            </span>
                          )}
                          {robot.batteryLevel < 20 && (
                            <span className="px-2 py-1 bg-yellow-100 dark:bg-yellow-900/30 text-yellow-700 dark:text-yellow-400 rounded-md flex items-center gap-1">
                              <Battery className="h-3 w-3" /> Low Battery ({robot.batteryLevel}%)
                            </span>
                          )}
                          {robot.errorCount > 0 && (
                            <span className="px-2 py-1 bg-red-100 dark:bg-red-900/30 text-red-700 dark:text-red-400 rounded-md flex items-center gap-1">
                              <AlertTriangle className="h-3 w-3" /> Errors ({robot.errorCount})
                            </span>
                          )}
                          {robot.temperature > 35 && (
                            <span className="px-2 py-1 bg-yellow-100 dark:bg-yellow-900/30 text-yellow-700 dark:text-yellow-400 rounded-md flex items-center gap-1">
                              <Thermometer className="h-3 w-3" /> High Temp ({robot.temperature}°C)
                            </span>
                          )}
                        </div>
                        <div className="text-sm text-muted-foreground">
                          Last ping: {new Date(robot.lastHeartbeat).toLocaleTimeString()}
                        </div>
                      </div>
                    </CardContent>
                  </Card>
                ))}
              </div>
            </CardContent>
          </Card>
        ) : (
          <Card>
            <CardContent className="p-8 text-center">
              <Bell className="h-12 w-12 mx-auto mb-4 text-green-500" />
              <h3 className="text-lg font-medium mb-2">All systems operational</h3>
              <p className="text-muted-foreground">
                No active alerts at this time. Your robot fleet is running smoothly.
              </p>
            </CardContent>
          </Card>
        )}
      </div>
    </MainLayout>
  );
};

export default Alerts;
