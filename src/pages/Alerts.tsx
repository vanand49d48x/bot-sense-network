
import { useState, useEffect } from "react";
import { MainLayout } from "@/components/layout/MainLayout";
import { Table, TableBody, TableCell, TableHead, TableHeader, TableRow } from "@/components/ui/table";
import { useRobots } from "@/hooks/useRobots";
import { mapSupabaseRobotToAppRobot } from "@/utils/robotMapper";
import { formatDistanceToNow, format } from "date-fns";
import { Button } from "@/components/ui/button";
import { AlertTriangle, BellOff, Bell } from "lucide-react";
import { Alert, AlertDescription, AlertTitle } from "@/components/ui/alert";
import { Badge } from "@/components/ui/badge";

interface RobotAlert {
  id: string;
  robotId: string;
  robotName: string;
  type: 'warning' | 'error' | 'offline' | 'battery';
  message: string;
  timestamp: string;
  resolved: boolean;
  notificationSent: boolean;
}

export default function Alerts() {
  const { robots: supabaseRobots, loading } = useRobots();
  const [alerts, setAlerts] = useState<RobotAlert[]>([]);
  
  // Map Supabase robots to application Robot type
  const robots = supabaseRobots.map(mapSupabaseRobotToAppRobot);

  // Generate alerts based on robot data
  useEffect(() => {
    if (robots.length === 0) return;
    
    const newAlerts: RobotAlert[] = [];
    
    robots.forEach(robot => {
      // Add alert for offline robots
      if (robot.status === 'offline') {
        newAlerts.push({
          id: `${robot.id}-offline-${robot.lastHeartbeat}`,
          robotId: robot.id,
          robotName: robot.name,
          type: 'offline',
          message: `Robot is offline`,
          timestamp: robot.lastHeartbeat,
          resolved: false,
          notificationSent: false
        });
      }
      
      // Add alert for warning status
      if (robot.status === 'warning') {
        newAlerts.push({
          id: `${robot.id}-warning-${robot.lastHeartbeat}`,
          robotId: robot.id,
          robotName: robot.name,
          type: 'warning',
          message: `Robot is in warning state`,
          timestamp: robot.lastHeartbeat,
          resolved: false,
          notificationSent: false
        });
      }
      
      // Add alert for low battery (below 20%)
      if (robot.batteryLevel < 20) {
        newAlerts.push({
          id: `${robot.id}-battery-${robot.lastHeartbeat}`,
          robotId: robot.id,
          robotName: robot.name,
          type: 'battery',
          message: `Low battery (${robot.batteryLevel}%)`,
          timestamp: robot.lastHeartbeat,
          resolved: false,
          notificationSent: false
        });
      }
      
      // Add alert for high temperature (above 40°C)
      if (robot.temperature > 40) {
        newAlerts.push({
          id: `${robot.id}-error-${robot.lastHeartbeat}`,
          robotId: robot.id,
          robotName: robot.name,
          type: 'error',
          message: `High temperature (${robot.temperature}°C)`,
          timestamp: robot.lastHeartbeat,
          resolved: false,
          notificationSent: false
        });
      }
    });
    
    setAlerts(newAlerts);
  }, [robots]);

  // Format the timestamp for display
  const formatTimestamp = (timestamp: string) => {
    try {
      const date = new Date(timestamp);
      return {
        relative: formatDistanceToNow(date, { addSuffix: true }),
        absolute: format(date, 'MMM d, yyyy HH:mm:ss')
      };
    } catch (e) {
      return { relative: 'Unknown', absolute: 'Unknown' };
    }
  };

  // Get alert type styles
  const getAlertTypeStyles = (type: string) => {
    switch (type) {
      case 'error': return "text-red-600 bg-red-50 border-red-200";
      case 'warning': return "text-amber-600 bg-amber-50 border-amber-200";
      case 'offline': return "text-gray-600 bg-gray-50 border-gray-200";
      case 'battery': return "text-orange-600 bg-orange-50 border-orange-200";
      default: return "text-gray-600 bg-gray-50 border-gray-200";
    }
  };
  
  const resolveAlert = (alertId: string) => {
    setAlerts(alerts.map(alert => 
      alert.id === alertId ? { ...alert, resolved: true } : alert
    ));
  };
  
  const toggleNotification = (alertId: string) => {
    setAlerts(alerts.map(alert => 
      alert.id === alertId ? { ...alert, notificationSent: !alert.notificationSent } : alert
    ));
  };

  if (loading) {
    return (
      <MainLayout>
        <div className="flex items-center justify-center min-h-[400px]">
          <div className="text-center">
            <div className="animate-pulse-slow">Loading alerts...</div>
          </div>
        </div>
      </MainLayout>
    );
  }

  // Filter out resolved alerts
  const activeAlerts = alerts.filter(alert => !alert.resolved);

  return (
    <MainLayout>
      <div className="container py-6">
        <h1 className="text-3xl font-bold tracking-tight mb-6">Alerts</h1>
        
        {activeAlerts.length > 0 ? (
          <div className="rounded-md border">
            <Table>
              <TableHeader>
                <TableRow>
                  <TableHead>Robot</TableHead>
                  <TableHead>Type</TableHead>
                  <TableHead>Message</TableHead>
                  <TableHead>Time</TableHead>
                  <TableHead>Notification</TableHead>
                  <TableHead>Actions</TableHead>
                </TableRow>
              </TableHeader>
              <TableBody>
                {activeAlerts.map((alert) => {
                  const time = formatTimestamp(alert.timestamp);
                  return (
                    <TableRow key={alert.id} className={getAlertTypeStyles(alert.type)}>
                      <TableCell className="font-medium">{alert.robotName}</TableCell>
                      <TableCell className="capitalize">{alert.type}</TableCell>
                      <TableCell>{alert.message}</TableCell>
                      <TableCell title={time.absolute}>{time.relative}</TableCell>
                      <TableCell>
                        <Badge variant={alert.notificationSent ? "default" : "outline"}>
                          {alert.notificationSent ? "Sent" : "Not Sent"}
                        </Badge>
                      </TableCell>
                      <TableCell>
                        <div className="flex gap-2">
                          <Button 
                            variant="outline" 
                            size="sm" 
                            onClick={() => toggleNotification(alert.id)}
                          >
                            {alert.notificationSent ? <BellOff className="h-4 w-4" /> : <Bell className="h-4 w-4" />}
                          </Button>
                          <Button 
                            variant="outline" 
                            size="sm" 
                            onClick={() => resolveAlert(alert.id)}
                          >
                            Resolve
                          </Button>
                        </div>
                      </TableCell>
                    </TableRow>
                  );
                })}
              </TableBody>
            </Table>
          </div>
        ) : (
          <div className="text-center p-12 border border-dashed rounded-lg">
            <BellOff className="h-12 w-12 mx-auto text-muted-foreground mb-4" />
            <h3 className="text-lg font-medium mb-2">No active alerts</h3>
            <p className="text-muted-foreground">
              All robot systems are operating normally
            </p>
          </div>
        )}
        
        <Alert className="mt-8">
          <AlertTriangle className="h-4 w-4" />
          <AlertTitle>About alerts</AlertTitle>
          <AlertDescription>
            Alerts are generated automatically based on robot status, battery levels, and temperature readings.
            The system checks for offline robots, warning states, low battery (&lt;20%), and high temperatures (&gt;40°C).
            Notification status can be toggled for each alert.
          </AlertDescription>
        </Alert>
      </div>
    </MainLayout>
  );
}
