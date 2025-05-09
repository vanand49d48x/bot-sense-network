import React, { useEffect, useState } from 'react';
import { supabase } from '@/integrations/supabase/client';
import { useToast } from '@/hooks/use-toast';
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";

type Alert = {
  id: string;
  robot_id: string;
  type: string;
  message: string;
  resolved: boolean;
  created_at: string;
  resolved_at: string | null;
};

type Robot = {
  id: string;
  name: string;
  type: string;
  user_id: string;
};

type User = {
  id: string;
  first_name: string | null;
  last_name: string | null;
};

const AlertManagement = () => {
  const [alerts, setAlerts] = useState<Alert[]>([]);
  const [robots, setRobots] = useState<Robot[]>([]);
  const [users, setUsers] = useState<User[]>([]);
  const [loading, setLoading] = useState(true);
  const { toast } = useToast();

  useEffect(() => {
    fetchAllData();
  }, []);

  const fetchAllData = async () => {
    try {
      setLoading(true);
      // Step 1: Fetch alerts
      const { data: alerts, error: alertsError } = await supabase
        .from('alerts')
        .select('*')
        .order('created_at', { ascending: false });
      if (alertsError) throw alertsError;
      setAlerts(alerts || []);

      // Step 2: Fetch robots for those alerts
      const robotIds = Array.from(new Set((alerts || []).map(a => a.robot_id).filter(Boolean)));
      let robots: Robot[] = [];
      if (robotIds.length > 0) {
        const { data: robotsData, error: robotsError } = await supabase
          .from('robots')
          .select('id, name, type, user_id')
          .in('id', robotIds);
        if (robotsError) throw robotsError;
        robots = robotsData || [];
        setRobots(robots);
      } else {
        setRobots([]);
      }

      // Step 3: Fetch users for those robots
      const userIds = Array.from(new Set((robots || []).map(r => r.user_id).filter(Boolean)));
      if (userIds.length > 0) {
        const { data: usersData, error: usersError } = await supabase
          .from('profiles')
          .select('id, first_name, last_name')
          .in('id', userIds);
        if (usersError) throw usersError;
        setUsers(usersData || []);
      } else {
        setUsers([]);
      }
    } catch (error: any) {
      toast({
        title: "Error fetching alerts",
        description: error.message,
        variant: "destructive",
      });
    } finally {
      setLoading(false);
    }
  };

  const handleResolveAlert = async (alertId: string) => {
    try {
      const { error } = await supabase
        .from('alerts')
        .update({
          resolved: true,
          resolved_at: new Date().toISOString(),
        })
        .eq('id', alertId);

      if (error) throw error;

      toast({
        title: "Alert resolved",
        description: "The alert has been marked as resolved.",
      });
      fetchAllData();
    } catch (error: any) {
      toast({
        title: "Error resolving alert",
        description: error.message,
        variant: "destructive",
      });
    }
  };

  const getAlertTypeColor = (type: string) => {
    switch (type.toLowerCase()) {
      case 'error':
        return 'bg-red-500';
      case 'warning':
        return 'bg-yellow-500';
      case 'info':
        return 'bg-blue-500';
      default:
        return 'bg-gray-500';
    }
  };

  // Map robots and users by id for quick lookup
  const robotMap: Record<string, Robot> = {};
  robots.forEach(robot => {
    robotMap[robot.id] = robot;
  });
  const userMap: Record<string, User> = {};
  users.forEach(user => {
    userMap[user.id] = user;
  });

  if (loading) {
    return <div>Loading alerts...</div>;
  }

  return (
    <div className="space-y-4">
      <div className="flex justify-between items-center">
        <h2 className="text-2xl font-bold">Alerts</h2>
      </div>

      <Table>
        <TableHeader>
          <TableRow>
            <TableHead>Type</TableHead>
            <TableHead>Message</TableHead>
            <TableHead>Robot</TableHead>
            <TableHead>Owner</TableHead>
            <TableHead>Created</TableHead>
            <TableHead>Status</TableHead>
            <TableHead>Actions</TableHead>
          </TableRow>
        </TableHeader>
        <TableBody>
          {alerts.map((alert) => {
            const robot = alert.robot_id ? robotMap[alert.robot_id] : null;
            const owner = robot && robot.user_id ? userMap[robot.user_id] : null;
            return (
              <TableRow key={alert.id}>
                <TableCell>
                  <Badge className={getAlertTypeColor(alert.type)}>
                    {alert.type}
                  </Badge>
                </TableCell>
                <TableCell>{alert.message}</TableCell>
                <TableCell>
                  {robot ? (
                    <>
                      {robot.name}
                      <br />
                      <span className="text-sm text-gray-500">{robot.type}</span>
                    </>
                  ) : (
                    <span className="text-gray-400">Unknown</span>
                  )}
                </TableCell>
                <TableCell>
                  {owner ? (
                    <>
                      {owner.first_name} {owner.last_name}
                    </>
                  ) : (
                    <span className="text-gray-400">Unknown</span>
                  )}
                </TableCell>
                <TableCell>
                  {new Date(alert.created_at).toLocaleString()}
                </TableCell>
                <TableCell>
                  <div>
                    <Badge className={alert.resolved ? 'bg-green-500' : 'bg-red-500'}>
                      {alert.resolved ? 'Resolved' : 'Active'}
                    </Badge>
                    {alert.resolved_at && (
                      <div className="text-sm text-gray-500">
                        Resolved: {new Date(alert.resolved_at).toLocaleString()}
                      </div>
                    )}
                  </div>
                </TableCell>
                <TableCell>
                  <div className="space-x-2">
                    {!alert.resolved && (
                      <Button
                        variant="outline"
                        size="sm"
                        onClick={() => handleResolveAlert(alert.id)}
                      >
                        Resolve
                      </Button>
                    )}
                  </div>
                </TableCell>
              </TableRow>
            );
          })}
        </TableBody>
      </Table>
    </div>
  );
};

export default AlertManagement; 