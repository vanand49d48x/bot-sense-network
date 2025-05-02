
import { MainLayout } from "@/components/layout/MainLayout";
import { useRobots } from "@/hooks/useRobots";
import { Robot } from "@/types/robot";
import { mapSupabaseRobotToAppRobot } from "@/utils/robotMapper";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Alert, AlertDescription, AlertTitle } from "@/components/ui/alert";
import { AlertTriangle, Bell, BellRing, Battery, Thermometer, Check, Mail, Phone } from "lucide-react";
import { useEffect, useState } from "react";
import { RobotStatusBadge } from "@/components/dashboard/RobotStatusBadge";
import { Button } from "@/components/ui/button";
import { toast } from "@/components/ui/sonner";
import { supabase } from "@/integrations/supabase/client";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";

const Alerts = () => {
  const { robots: supabaseRobots, loading } = useRobots();
  const [robots, setRobots] = useState<Robot[]>([]);
  const [selectedFilter, setSelectedFilter] = useState<string | null>(null);
  
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

  // Get filtered robots based on selected filter
  const getFilteredRobots = () => {
    switch (selectedFilter) {
      case 'battery':
        return lowBatteryRobots;
      case 'offline':
        return offlineRobots;
      case 'errors':
        return errorRobots;
      case 'temperature':
        return highTempRobots;
      default:
        return robotsWithAlerts;
    }
  };

  const getFilterTitle = () => {
    switch (selectedFilter) {
      case 'battery':
        return 'Low Battery Robots';
      case 'offline':
        return 'Offline Robots';
      case 'errors':
        return 'Robots with Errors';
      case 'temperature':
        return 'Robots with High Temperature';
      default:
        return 'All Alerts';
    }
  };

  // Handle card click
  const handleCardClick = (filter: string) => {
    setSelectedFilter(selectedFilter === filter ? null : filter);
  };

  // Handle notification actions
  const handleEmailNotification = (robotId: string, robotName: string) => {
    toast.success(`Email notification sent for ${robotName}`);
  };

  const handleSmsNotification = (robotId: string, robotName: string) => {
    toast.success(`SMS notification sent for ${robotName}`);
  };

  // Handle resolving an alert
  const handleResolveAlert = async (robotId: string, robotName: string) => {
    try {
      // This would normally update the alert status in the database
      toast.success(`Alert for ${robotName} marked as resolved`);
    } catch (error) {
      toast.error("Failed to resolve alert");
      console.error("Error resolving alert:", error);
    }
  };

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
          <Card 
            className={`border-l-4 ${lowBatteryRobots.length > 0 ? 'border-l-yellow-500' : 'border-l-green-500'} cursor-pointer hover:shadow-md transition-all ${selectedFilter === 'battery' ? 'ring-2 ring-primary ring-offset-2' : ''}`}
            onClick={() => handleCardClick('battery')}
          >
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
          
          <Card 
            className={`border-l-4 ${offlineRobots.length > 0 ? 'border-l-red-500' : 'border-l-green-500'} cursor-pointer hover:shadow-md transition-all ${selectedFilter === 'offline' ? 'ring-2 ring-primary ring-offset-2' : ''}`}
            onClick={() => handleCardClick('offline')}
          >
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
          
          <Card 
            className={`border-l-4 ${errorRobots.length > 0 ? 'border-l-red-500' : 'border-l-green-500'} cursor-pointer hover:shadow-md transition-all ${selectedFilter === 'errors' ? 'ring-2 ring-primary ring-offset-2' : ''}`}
            onClick={() => handleCardClick('errors')}
          >
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
          
          <Card 
            className={`border-l-4 ${highTempRobots.length > 0 ? 'border-l-yellow-500' : 'border-l-green-500'} cursor-pointer hover:shadow-md transition-all ${selectedFilter === 'temperature' ? 'ring-2 ring-primary ring-offset-2' : ''}`}
            onClick={() => handleCardClick('temperature')}
          >
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
        
        {selectedFilter && getFilteredRobots().length === 0 && (
          <Alert>
            <AlertTitle>No robots found</AlertTitle>
            <AlertDescription>
              No robots match the selected filter.
            </AlertDescription>
          </Alert>
        )}
        
        {selectedFilter ? (
          <Card>
            <CardHeader>
              <div className="flex items-center justify-between">
                <CardTitle className="flex items-center gap-2">
                  <AlertTriangle className="h-5 w-5" />
                  <span>{getFilterTitle()}</span>
                </CardTitle>
                <Button 
                  variant="ghost" 
                  className="text-sm" 
                  onClick={() => setSelectedFilter(null)}
                >
                  View All Alerts
                </Button>
              </div>
              <CardDescription>
                {getFilteredRobots().length} {getFilteredRobots().length === 1 ? 'robot' : 'robots'} with issues
              </CardDescription>
            </CardHeader>
            <CardContent>
              <Table>
                <TableHeader>
                  <TableRow>
                    <TableHead>Robot</TableHead>
                    <TableHead>Status</TableHead>
                    <TableHead>Issues</TableHead>
                    <TableHead>Last Ping</TableHead>
                    <TableHead>Actions</TableHead>
                    <TableHead>Resolve</TableHead>
                  </TableRow>
                </TableHeader>
                <TableBody>
                  {getFilteredRobots().map(robot => (
                    <TableRow key={robot.id}>
                      <TableCell className="font-medium">{robot.name}</TableCell>
                      <TableCell><RobotStatusBadge status={robot.status} /></TableCell>
                      <TableCell>
                        <div className="flex flex-wrap gap-1">
                          {robot.status === 'offline' && (
                            <span className="px-2 py-0.5 text-xs bg-red-100 dark:bg-red-900/30 text-red-700 dark:text-red-400 rounded-md flex items-center gap-1">
                              <BellRing className="h-3 w-3" /> Offline
                            </span>
                          )}
                          {robot.batteryLevel < 20 && (
                            <span className="px-2 py-0.5 text-xs bg-yellow-100 dark:bg-yellow-900/30 text-yellow-700 dark:text-yellow-400 rounded-md flex items-center gap-1">
                              <Battery className="h-3 w-3" /> {robot.batteryLevel}%
                            </span>
                          )}
                          {robot.errorCount > 0 && (
                            <span className="px-2 py-0.5 text-xs bg-red-100 dark:bg-red-900/30 text-red-700 dark:text-red-400 rounded-md flex items-center gap-1">
                              <AlertTriangle className="h-3 w-3" /> {robot.errorCount}
                            </span>
                          )}
                          {robot.temperature > 35 && (
                            <span className="px-2 py-0.5 text-xs bg-yellow-100 dark:bg-yellow-900/30 text-yellow-700 dark:text-yellow-400 rounded-md flex items-center gap-1">
                              <Thermometer className="h-3 w-3" /> {robot.temperature}°C
                            </span>
                          )}
                        </div>
                      </TableCell>
                      <TableCell>{new Date(robot.lastHeartbeat).toLocaleTimeString()}</TableCell>
                      <TableCell>
                        <div className="flex gap-2">
                          <Button 
                            size="sm" 
                            variant="outline"
                            onClick={() => handleEmailNotification(robot.id, robot.name)}
                          >
                            <Mail className="h-3 w-3 mr-1" /> Email
                          </Button>
                          <Button 
                            size="sm" 
                            variant="outline"
                            onClick={() => handleSmsNotification(robot.id, robot.name)}
                          >
                            <Phone className="h-3 w-3 mr-1" /> Text
                          </Button>
                        </div>
                      </TableCell>
                      <TableCell>
                        <Button 
                          size="sm" 
                          variant="ghost"
                          className="text-green-600 hover:text-green-800 dark:text-green-400 dark:hover:text-green-200"
                          onClick={() => handleResolveAlert(robot.id, robot.name)}
                        >
                          <Check className="h-4 w-4" />
                        </Button>
                      </TableCell>
                    </TableRow>
                  ))}
                </TableBody>
              </Table>
            </CardContent>
          </Card>
        ) : (
          robotsWithAlerts.length > 0 ? (
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
                <Table>
                  <TableHeader>
                    <TableRow>
                      <TableHead>Robot</TableHead>
                      <TableHead>Status</TableHead>
                      <TableHead>Issues</TableHead>
                      <TableHead>Last Ping</TableHead>
                      <TableHead>Actions</TableHead>
                      <TableHead>Resolve</TableHead>
                    </TableRow>
                  </TableHeader>
                  <TableBody>
                    {robotsWithAlerts.map(robot => (
                      <TableRow key={robot.id}>
                        <TableCell className="font-medium">{robot.name}</TableCell>
                        <TableCell><RobotStatusBadge status={robot.status} /></TableCell>
                        <TableCell>
                          <div className="flex flex-wrap gap-1">
                            {robot.status === 'offline' && (
                              <span className="px-2 py-0.5 text-xs bg-red-100 dark:bg-red-900/30 text-red-700 dark:text-red-400 rounded-md flex items-center gap-1">
                                <BellRing className="h-3 w-3" /> Offline
                              </span>
                            )}
                            {robot.batteryLevel < 20 && (
                              <span className="px-2 py-0.5 text-xs bg-yellow-100 dark:bg-yellow-900/30 text-yellow-700 dark:text-yellow-400 rounded-md flex items-center gap-1">
                                <Battery className="h-3 w-3" /> {robot.batteryLevel}%
                              </span>
                            )}
                            {robot.errorCount > 0 && (
                              <span className="px-2 py-0.5 text-xs bg-red-100 dark:bg-red-900/30 text-red-700 dark:text-red-400 rounded-md flex items-center gap-1">
                                <AlertTriangle className="h-3 w-3" /> {robot.errorCount}
                              </span>
                            )}
                            {robot.temperature > 35 && (
                              <span className="px-2 py-0.5 text-xs bg-yellow-100 dark:bg-yellow-900/30 text-yellow-700 dark:text-yellow-400 rounded-md flex items-center gap-1">
                                <Thermometer className="h-3 w-3" /> {robot.temperature}°C
                              </span>
                            )}
                          </div>
                        </TableCell>
                        <TableCell>{new Date(robot.lastHeartbeat).toLocaleTimeString()}</TableCell>
                        <TableCell>
                          <div className="flex gap-2">
                            <Button 
                              size="sm" 
                              variant="outline"
                              onClick={() => handleEmailNotification(robot.id, robot.name)}
                            >
                              <Mail className="h-3 w-3 mr-1" /> Email
                            </Button>
                            <Button 
                              size="sm" 
                              variant="outline"
                              onClick={() => handleSmsNotification(robot.id, robot.name)}
                            >
                              <Phone className="h-3 w-3 mr-1" /> Text
                            </Button>
                          </div>
                        </TableCell>
                        <TableCell>
                          <Button 
                            size="sm" 
                            variant="ghost"
                            className="text-green-600 hover:text-green-800 dark:text-green-400 dark:hover:text-green-200"
                            onClick={() => handleResolveAlert(robot.id, robot.name)}
                          >
                            <Check className="h-4 w-4" />
                          </Button>
                        </TableCell>
                      </TableRow>
                    ))}
                  </TableBody>
                </Table>
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
          )
        )}
      </div>
    </MainLayout>
  );
};

export default Alerts;
