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
import { Input } from "@/components/ui/input";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { useDebounce } from "../../hooks/useDebounce";

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
  email?: string;
};

type SupabaseUser = {
  id: string;
  first_name: string | null;
  last_name: string | null;
  user: {
    email: string;
  } | null;
};

const AlertManagement = () => {
  const [alerts, setAlerts] = useState<Alert[]>([]);
  const [robots, setRobots] = useState<Robot[]>([]);
  const [users, setUsers] = useState<User[]>([]);
  const [loading, setLoading] = useState(true);
  const { toast } = useToast();
  const [filter, setFilter] = useState({ userId: 'all', robotId: 'all', type: 'all' });
  const [search, setSearch] = useState('');
  const debouncedSearch = useDebounce(search, 300);

  useEffect(() => {
    fetchAllData();
  }, [filter, debouncedSearch]);

  const fetchAllData = async () => {
    try {
      setLoading(true);
      // Step 1: Fetch alerts
      let alertQuery = supabase
        .from('alerts')
        .select('*')
        .order('created_at', { ascending: false });
      
      if (filter.userId && filter.userId !== 'all') {
        alertQuery = alertQuery.eq('user_id', filter.userId);
      }
      if (filter.robotId && filter.robotId !== 'all') {
        alertQuery = alertQuery.eq('robot_id', filter.robotId);
      }
      if (filter.type && filter.type !== 'all') {
        alertQuery = alertQuery.eq('type', filter.type);
      }
      if (debouncedSearch) {
        alertQuery = alertQuery.ilike('message', `%${debouncedSearch}%`);
      }

      const { data: alerts, error: alertsError } = await alertQuery;
      if (alertsError) throw alertsError;

      // Step 2: Fetch robots for the dropdown
      const { data: robots, error: robotsError } = await supabase
        .from('robots')
        .select('id, name, type, user_id');
      if (robotsError) throw robotsError;

      // Step 3: Fetch users for the dropdown
      const { data: users, error: usersError } = await supabase
        .from('profiles')
        .select('id, first_name, last_name, email');
      if (usersError) throw usersError;

      setAlerts(alerts || []);
      setRobots(robots || []);
      setUsers(users || []);
    } catch (error: any) {
      toast({
        title: "Error fetching data",
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
      <div className="flex flex-col md:flex-row md:justify-between md:items-center gap-4">
        <h2 className="text-2xl font-bold">Alerts</h2>
        <div className="flex flex-wrap gap-2 items-center bg-muted/50 p-3 rounded-md shadow-sm">
          <Select value={filter.userId} onValueChange={v => setFilter(f => ({ ...f, userId: v }))}>
            <SelectTrigger className="w-48">
              <SelectValue placeholder="Filter by User" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="all">All Users</SelectItem>
              {users.map(u => (
                <SelectItem key={u.id} value={u.id}>
                  {u.first_name} {u.last_name} ({u.email || 'No email'})
                </SelectItem>
              ))}
            </SelectContent>
          </Select>

          <Select value={filter.robotId} onValueChange={v => setFilter(f => ({ ...f, robotId: v }))}>
            <SelectTrigger className="w-48">
              <SelectValue placeholder="Filter by Robot" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="all">All Robots</SelectItem>
              {robots.map(r => (
                <SelectItem key={r.id} value={r.id}>
                  {r.name} ({r.type})
                </SelectItem>
              ))}
            </SelectContent>
          </Select>

          <Select value={filter.type} onValueChange={v => setFilter(f => ({ ...f, type: v }))}>
            <SelectTrigger className="w-40">
              <SelectValue placeholder="Alert Type" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="all">All Types</SelectItem>
              <SelectItem value="error">Error</SelectItem>
              <SelectItem value="warning">Warning</SelectItem>
              <SelectItem value="info">Info</SelectItem>
            </SelectContent>
          </Select>

          <Input
            className="w-48"
            placeholder="Search by Message"
            value={search}
            onChange={e => setSearch(e.target.value)}
          />
          <Button 
            variant="outline" 
            onClick={() => { 
              setFilter({ userId: 'all', robotId: 'all', type: 'all' }); 
              setSearch(''); 
            }}
          >
            Clear
          </Button>
        </div>
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