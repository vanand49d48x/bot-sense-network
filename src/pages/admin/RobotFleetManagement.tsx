
import { useState, useEffect } from "react";
import { useToast } from "@/hooks/use-toast";
import { supabase } from "@/integrations/supabase/client";
import { Loader2, Search, Bot, BarChart, Battery, Thermometer } from "lucide-react";
import { Input } from "@/components/ui/input";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { Badge } from "@/components/ui/badge";

interface Robot {
  id: string;
  name: string;
  type: string;
  status: string;
  battery_level: number;
  temperature: number;
  last_ping: string | null;
  user_id: string;
  user_email?: string;
}

export default function RobotFleetManagement() {
  const { toast } = useToast();
  const [robots, setRobots] = useState<Robot[]>([]);
  const [loading, setLoading] = useState(true);
  const [searchQuery, setSearchQuery] = useState("");
  const [statusFilter, setStatusFilter] = useState<string>("all");
  
  useEffect(() => {
    fetchRobots();
  }, []);

  const fetchRobots = async () => {
    setLoading(true);
    try {
      // Get all robots
      const { data, error } = await supabase
        .from('robots')
        .select(`
          *,
          profiles:user_id (
            first_name,
            last_name
          )
        `);

      if (error) throw error;

      // Transform the data to include user info
      const robotsWithUserInfo = data.map((robot: any) => ({
        ...robot,
        user_email: robot.profiles 
          ? `${robot.profiles.first_name || ''} ${robot.profiles.last_name || ''}`.trim() || 'Unknown user'
          : 'Unknown user'
      }));

      setRobots(robotsWithUserInfo);
    } catch (error: any) {
      console.error("Error fetching robots:", error);
      toast({
        title: "Failed to load robots",
        description: error.message || "Could not retrieve robot data",
        variant: "destructive",
      });
    } finally {
      setLoading(false);
    }
  };

  const formatDate = (dateString: string | null) => {
    if (!dateString) return "Never";
    return new Date(dateString).toLocaleDateString() + " " + 
           new Date(dateString).toLocaleTimeString();
  };

  const getStatusBadge = (status: string) => {
    switch (status.toLowerCase()) {
      case 'online':
        return <Badge className="bg-green-500">Online</Badge>;
      case 'offline':
        return <Badge variant="outline" className="text-gray-500">Offline</Badge>;
      case 'error':
        return <Badge className="bg-red-500">Error</Badge>;
      case 'maintenance':
        return <Badge className="bg-yellow-500">Maintenance</Badge>;
      default:
        return <Badge variant="outline">{status}</Badge>;
    }
  };

  const filteredRobots = robots.filter(robot => {
    const matchesSearch = 
      robot.name.toLowerCase().includes(searchQuery.toLowerCase()) ||
      robot.type.toLowerCase().includes(searchQuery.toLowerCase()) ||
      (robot.user_email && robot.user_email.toLowerCase().includes(searchQuery.toLowerCase()));
    
    const matchesStatus = statusFilter === 'all' || robot.status.toLowerCase() === statusFilter.toLowerCase();
    
    return matchesSearch && matchesStatus;
  });

  return (
    <div className="space-y-6">
      <div>
        <h1 className="text-3xl font-bold tracking-tight">Robot Fleet Management</h1>
        <p className="text-muted-foreground">
          Monitor and manage all robots across the platform
        </p>
      </div>

      <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-4">
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Total Robots</CardTitle>
            <Bot className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{robots.length}</div>
          </CardContent>
        </Card>
        
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Online Robots</CardTitle>
            <BarChart className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">
              {robots.filter(r => r.status.toLowerCase() === 'online').length}
            </div>
            <p className="text-xs text-muted-foreground">
              {Math.round(robots.filter(r => r.status.toLowerCase() === 'online').length / (robots.length || 1) * 100)}% of fleet
            </p>
          </CardContent>
        </Card>
        
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Avg Battery</CardTitle>
            <Battery className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">
              {Math.round(robots.reduce((sum, robot) => sum + (robot.battery_level || 0), 0) / (robots.length || 1))}%
            </div>
          </CardContent>
        </Card>
        
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Avg Temperature</CardTitle>
            <Thermometer className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">
              {Math.round(robots.reduce((sum, robot) => sum + (robot.temperature || 0), 0) / (robots.length || 1))}°C
            </div>
          </CardContent>
        </Card>
      </div>

      <Card>
        <CardHeader>
          <CardTitle>Robot Fleet</CardTitle>
          <CardDescription>
            Monitor and manage all robots in the system
          </CardDescription>
        </CardHeader>
        <CardContent>
          <div className="flex flex-col md:flex-row gap-4 mb-4">
            <div className="relative flex-1">
              <Search className="absolute left-2 top-2.5 h-4 w-4 text-muted-foreground" />
              <Input 
                placeholder="Search robots by name, type, or user..."
                className="pl-8"
                value={searchQuery}
                onChange={(e) => setSearchQuery(e.target.value)}
              />
            </div>
            <div className="w-full md:w-48">
              <Select value={statusFilter} onValueChange={setStatusFilter}>
                <SelectTrigger>
                  <SelectValue placeholder="Filter by status" />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="all">All Statuses</SelectItem>
                  <SelectItem value="online">Online</SelectItem>
                  <SelectItem value="offline">Offline</SelectItem>
                  <SelectItem value="error">Error</SelectItem>
                  <SelectItem value="maintenance">Maintenance</SelectItem>
                </SelectContent>
              </Select>
            </div>
          </div>

          {loading ? (
            <div className="flex items-center justify-center py-8">
              <Loader2 className="h-8 w-8 animate-spin text-primary" />
              <span className="ml-2">Loading robots...</span>
            </div>
          ) : (
            <div className="rounded-md border overflow-hidden">
              <Table>
                <TableHeader>
                  <TableRow>
                    <TableHead>Name</TableHead>
                    <TableHead>Type</TableHead>
                    <TableHead>Status</TableHead>
                    <TableHead>Battery</TableHead>
                    <TableHead>Temperature</TableHead>
                    <TableHead>Last Ping</TableHead>
                    <TableHead>Owner</TableHead>
                  </TableRow>
                </TableHeader>
                <TableBody>
                  {filteredRobots.length > 0 ? (
                    filteredRobots.map((robot) => (
                      <TableRow key={robot.id}>
                        <TableCell>{robot.name}</TableCell>
                        <TableCell>{robot.type}</TableCell>
                        <TableCell>{getStatusBadge(robot.status)}</TableCell>
                        <TableCell>
                          {typeof robot.battery_level === 'number' ? (
                            <div className="flex items-center">
                              <div className="w-16 bg-muted rounded-full h-2 mr-2">
                                <div 
                                  className={`h-2 rounded-full ${
                                    robot.battery_level > 60 ? 'bg-green-500' : 
                                    robot.battery_level > 20 ? 'bg-yellow-500' : 
                                    'bg-red-500'
                                  }`} 
                                  style={{ width: `${robot.battery_level || 0}%` }}
                                />
                              </div>
                              <span>{robot.battery_level}%</span>
                            </div>
                          ) : (
                            'N/A'
                          )}
                        </TableCell>
                        <TableCell>
                          {typeof robot.temperature === 'number' ? (
                            `${robot.temperature}°C`
                          ) : (
                            'N/A'
                          )}
                        </TableCell>
                        <TableCell>{formatDate(robot.last_ping)}</TableCell>
                        <TableCell>{robot.user_email}</TableCell>
                      </TableRow>
                    ))
                  ) : (
                    <TableRow>
                      <TableCell colSpan={7} className="h-24 text-center">
                        No robots found matching the criteria
                      </TableCell>
                    </TableRow>
                  )}
                </TableBody>
              </Table>
            </div>
          )}
        </CardContent>
      </Card>
    </div>
  );
}
