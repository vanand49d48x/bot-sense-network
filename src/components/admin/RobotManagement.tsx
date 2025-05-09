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

  useEffect(() => {
    fetchRobots();
  }, []);

  const fetchRobots = async () => {
    try {
      setLoading(true);
      const { data: robots, error } = await supabase
        .from('robots')
        .select(`
          *,
          user:user_id (
            id,
            first_name,
            last_name
          )
        `);

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
      <div className="flex justify-between items-center">
        <h2 className="text-2xl font-bold">Robots</h2>
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
                {robot.user?.profile?.first_name} {robot.user?.profile?.last_name}
                <br />
                <span className="text-sm text-gray-500">{robot.user?.email}</span>
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