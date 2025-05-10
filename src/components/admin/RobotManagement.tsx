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
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Badge } from "@/components/ui/badge";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { useDebounce } from "../../hooks/useDebounce";
import type { ReactNode } from 'react';

type Robot = {
  id: string;
  name: string;
  type: string;
  status: string;
  last_ping: string | null;
  battery_level: number | null;
  temperature: number | null;
  user_id: string;
  created_at: string;
  user: {
    email: string;
    profile: {
      first_name: string | null;
      last_name: string | null;
    } | null;
  } | null;
};

const RobotManagement = () => {
  const [robots, setRobots] = useState<Robot[]>([]);
  const [loading, setLoading] = useState(true);
  const { toast } = useToast();
  const [users, setUsers] = useState<any[]>([]);
  const [robotTypes, setRobotTypes] = useState<string[]>([]);
  const [filter, setFilter] = useState({ userId: 'all', name: '', type: 'all' });
  const [search, setSearch] = useState('');
  const debouncedSearch = useDebounce(search, 300);

  useEffect(() => {
    fetchUsersAndTypes();
  }, []);

  useEffect(() => {
    fetchRobots();
  }, [filter, debouncedSearch]);

  const fetchUsersAndTypes = async () => {
    // Fetch all users for dropdown
    const { data: profiles } = await supabase.from('profiles').select('id, email, first_name, last_name');
    setUsers(profiles || []);
    // Fetch all robot types for dropdown
    const { data: robots } = await supabase.from('robots').select('type');
    const types = Array.from(new Set((robots || []).map((r: any) => r.type))).filter(Boolean);
    setRobotTypes(types);
  };

  const fetchRobots = async () => {
    try {
      setLoading(true);
      let query = supabase
        .from('robots')
        .select(`*, user:profiles!inner(id, email, first_name, last_name)`);
      if (filter.userId && filter.userId !== 'all') query = query.eq('user_id', filter.userId);
      if (filter.type && filter.type !== 'all') query = query.eq('type', filter.type);
      if (debouncedSearch) query = query.ilike('name', `%${debouncedSearch}%`);
      const { data: robots, error } = await query;
      if (error) throw error;
      setRobots(robots || []);
    } catch (error: any) {
      toast({
        title: "Error fetching robots",
        description: error.message,
        variant: "destructive",
      });
    } finally {
      setLoading(false);
    }
  };

  const handleDeleteRobot = async (robotId: string) => {
    try {
      const { error } = await supabase
        .from('robots')
        .delete()
        .eq('id', robotId);

      if (error) throw error;

      toast({
        title: "Robot deleted",
        description: "The robot has been successfully deleted.",
      });
      fetchRobots();
    } catch (error: any) {
      toast({
        title: "Error deleting robot",
        description: error.message,
        variant: "destructive",
      });
    }
  };

  const getStatusColor = (status: string) => {
    switch (status.toLowerCase()) {
      case 'online':
        return 'bg-green-500';
      case 'offline':
        return 'bg-red-500';
      case 'maintenance':
        return 'bg-yellow-500';
      default:
        return 'bg-gray-500';
    }
  };

  if (loading) {
    return <div>Loading robots...</div>;
  }

  return (
    <div className="space-y-4">
      <div className="flex flex-col md:flex-row md:justify-between md:items-center gap-4">
        <h2 className="text-2xl font-bold">Robots</h2>
        <div className="flex flex-wrap gap-2 items-center bg-muted/50 p-3 rounded-md shadow-sm">
          <Select value={filter.userId} onValueChange={v => setFilter(f => ({ ...f, userId: v }))}>
            <SelectTrigger className="w-48">
              <SelectValue placeholder="Filter by User" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="all">All Users</SelectItem>
              {users.map(u => (
                <SelectItem key={u.id} value={u.id}>{u.first_name} {u.last_name} ({u.email})</SelectItem>
              ))}
            </SelectContent>
          </Select>
          <Select value={filter.type} onValueChange={v => setFilter(f => ({ ...f, type: v }))}>
            <SelectTrigger className="w-40">
              <SelectValue placeholder="Robot Type" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="all">All Types</SelectItem>
              {robotTypes.map(type => (
                <SelectItem key={type} value={type}>{type}</SelectItem>
              ))}
            </SelectContent>
          </Select>
          <Input
            className="w-48"
            placeholder="Search by Name"
            value={search}
            onChange={e => setSearch(e.target.value)}
          />
          <Button variant="outline" onClick={() => { setFilter({ userId: 'all', name: '', type: 'all' }); setSearch(''); }}>Clear</Button>
        </div>
        <Dialog>
          <DialogTrigger asChild>
            <Button>Add Robot</Button>
          </DialogTrigger>
          <DialogContent>
            <DialogHeader>
              <DialogTitle>Add New Robot</DialogTitle>
              <DialogDescription>
                Create a new robot and assign it to a user.
              </DialogDescription>
            </DialogHeader>
            <form className="space-y-4">
              <div className="space-y-2">
                <Label htmlFor="name">Robot Name</Label>
                <Input id="name" placeholder="Enter robot name" />
              </div>
              <div className="space-y-2">
                <Label htmlFor="type">Robot Type</Label>
                <Input id="type" placeholder="Enter robot type" />
              </div>
              <div className="space-y-2">
                <Label htmlFor="user_id">User Email</Label>
                <Input id="user_id" type="email" placeholder="user@example.com" />
              </div>
              <Button type="submit">Create Robot</Button>
            </form>
          </DialogContent>
        </Dialog>
      </div>

      <Table>
        <TableHeader>
          <TableRow>
            <TableHead>Name</TableHead>
            <TableHead>Type</TableHead>
            <TableHead>Status</TableHead>
            <TableHead>Owner</TableHead>
            <TableHead>Last Ping</TableHead>
            <TableHead>Battery</TableHead>
            <TableHead>Temperature</TableHead>
            <TableHead>Actions</TableHead>
          </TableRow>
        </TableHeader>
        <TableBody>
          {robots.map((robot) => (
            <TableRow key={robot.id}>
              <TableCell>{robot.name}</TableCell>
              <TableCell>{robot.type}</TableCell>
              <TableCell>
                <Badge className={getStatusColor(robot.status)}>
                  {robot.status}
                </Badge>
              </TableCell>
              <TableCell>
                {robot.user && typeof robot.user === 'object' ? (
                  'first_name' in robot.user || 'last_name' in robot.user ? (
                    <>
                      {('first_name' in robot.user ? robot.user.first_name : '') as ReactNode} {('last_name' in robot.user ? robot.user.last_name : '') as ReactNode}
                      <br />
                      <span className="text-sm text-gray-500">{robot.user.email as ReactNode}</span>
                    </>
                  ) : ('profile' in robot.user && robot.user.profile) ? (
                    <>
                      {robot.user.profile.first_name as ReactNode || ''} {robot.user.profile.last_name as ReactNode || ''}
                      <br />
                      <span className="text-sm text-gray-500">{robot.user.email as ReactNode}</span>
                    </>
                  ) : (
                    <span className="text-sm text-gray-400">Unknown</span>
                  )
                ) : (
                  <span className="text-sm text-gray-400">Unknown</span>
                )}
              </TableCell>
              <TableCell>
                {robot.last_ping
                  ? new Date(robot.last_ping).toLocaleString()
                  : 'Never'}
              </TableCell>
              <TableCell>
                {robot.battery_level !== null ? `${robot.battery_level}%` : 'N/A'}
              </TableCell>
              <TableCell>
                {robot.temperature !== null ? `${robot.temperature}°C` : 'N/A'}
              </TableCell>
              <TableCell>
                <div className="space-x-2">
                  <Button
                    variant="outline"
                    size="sm"
                    onClick={() => handleDeleteRobot(robot.id)}
                  >
                    Delete
                  </Button>
                </div>
              </TableCell>
            </TableRow>
          ))}
        </TableBody>
      </Table>
    </div>
  );
};

export default RobotManagement; 