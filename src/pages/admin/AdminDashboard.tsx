
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { useToast } from "@/hooks/use-toast";
import { useAuth } from "@/context/AuthContext";
import { useEffect, useState } from "react";
import { supabase } from "@/integrations/supabase/client";
import { Loader2, Users, Bot, Bell, UserCheck } from "lucide-react";
import { Progress } from "@/components/ui/progress";

interface AdminStats {
  totalUsers: number;
  totalRobots: number;
  totalAlerts: number;
  activeUsers: number;
  newUsers24h: number;
}

export default function AdminDashboard() {
  const { user } = useAuth();
  const { toast } = useToast();
  const [loading, setLoading] = useState(true);
  const [stats, setStats] = useState<AdminStats>({
    totalUsers: 0,
    totalRobots: 0,
    totalAlerts: 0,
    activeUsers: 0,
    newUsers24h: 0,
  });

  useEffect(() => {
    const fetchAdminStats = async () => {
      setLoading(true);
      try {
        // Get total users count
        const { count: totalUsers, error: usersError } = await supabase
          .from("profiles")
          .select("id", { count: "exact", head: true });

        if (usersError) throw usersError;

        // Get total robots count
        const { count: totalRobots, error: robotsError } = await supabase
          .from("robots")
          .select("id", { count: "exact", head: true });

        if (robotsError) throw robotsError;

        // Get total alerts count
        const { count: totalAlerts, error: alertsError } = await supabase
          .from("alerts")
          .select("id", { count: "exact", head: true });

        if (alertsError) throw alertsError;

        // Get active users (users with robots that pinged in the last 24h)
        // Fix: Using a different approach to get unique user_ids
        const { data: activeRobots, error: activeUsersError } = await supabase
          .from("robots")
          .select("user_id")
          .gte("last_ping", new Date(Date.now() - 24 * 60 * 60 * 1000).toISOString());

        if (activeUsersError) throw activeUsersError;

        // Count unique active users
        const activeUserIds = new Set(activeRobots?.map(robot => robot.user_id) || []);
        const activeUsers = activeUserIds.size;

        // Get new users in the last 24h
        const { count: newUsers24h, error: newUsersError } = await supabase
          .from("profiles")
          .select("id", { count: "exact", head: true })
          .gte("created_at", new Date(Date.now() - 24 * 60 * 60 * 1000).toISOString());

        if (newUsersError) throw newUsersError;

        setStats({
          totalUsers: totalUsers || 0,
          totalRobots: totalRobots || 0,
          totalAlerts: totalAlerts || 0,
          activeUsers: activeUsers || 0,
          newUsers24h: newUsers24h || 0,
        });
      } catch (error) {
        console.error("Error fetching admin stats:", error);
        toast({
          title: "Error fetching data",
          description: "Could not load admin dashboard statistics",
          variant: "destructive",
        });
      } finally {
        setLoading(false);
      }
    };

    fetchAdminStats();
  }, [toast]);

  if (loading) {
    return (
      <div className="flex items-center justify-center min-h-[400px]">
        <Loader2 className="h-8 w-8 animate-spin text-primary" />
        <span className="ml-2">Loading stats...</span>
      </div>
    );
  }

  return (
    <div className="space-y-6">
      <div>
        <h1 className="text-3xl font-bold tracking-tight">Admin Dashboard</h1>
        <p className="text-muted-foreground">
          System-wide statistics and administration
        </p>
      </div>

      <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-4">
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Total Users</CardTitle>
            <Users className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{stats.totalUsers}</div>
            <p className="text-xs text-muted-foreground">
              {stats.newUsers24h} new in the last 24h
            </p>
          </CardContent>
        </Card>
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Total Robots</CardTitle>
            <Bot className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{stats.totalRobots}</div>
            <p className="text-xs text-muted-foreground">
              ~{Math.round(stats.totalRobots / (stats.totalUsers || 1) * 10) / 10} per user
            </p>
          </CardContent>
        </Card>
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Active Users</CardTitle>
            <UserCheck className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{stats.activeUsers}</div>
            <div className="mt-2 space-y-1">
              <p className="text-xs text-muted-foreground">
                {Math.round(stats.activeUsers / (stats.totalUsers || 1) * 100)}% of total users
              </p>
              <Progress value={stats.activeUsers / (stats.totalUsers || 1) * 100} />
            </div>
          </CardContent>
        </Card>
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Total Alerts</CardTitle>
            <Bell className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{stats.totalAlerts}</div>
            <p className="text-xs text-muted-foreground">
              ~{Math.round(stats.totalAlerts / (stats.totalRobots || 1) * 10) / 10} per robot
            </p>
          </CardContent>
        </Card>
      </div>

      <h2 className="text-xl font-semibold mt-8">Quick Actions</h2>
      <div className="grid gap-4 md:grid-cols-2">
        <Card>
          <CardHeader>
            <CardTitle>User Management</CardTitle>
            <CardDescription>
              View and manage all system users
            </CardDescription>
          </CardHeader>
          <CardContent>
            <div className="space-y-2">
              <div className="flex justify-between text-sm">
                <span>Total users:</span>
                <span className="font-medium">{stats.totalUsers}</span>
              </div>
              <div className="flex justify-between text-sm">
                <span>New users (24h):</span>
                <span className="font-medium">{stats.newUsers24h}</span>
              </div>
              <div className="flex justify-between text-sm">
                <span>Active users:</span>
                <span className="font-medium">{stats.activeUsers}</span>
              </div>
            </div>
          </CardContent>
        </Card>
        
        <Card>
          <CardHeader>
            <CardTitle>Robot Fleet</CardTitle>
            <CardDescription>
              Manage all robots across the platform
            </CardDescription>
          </CardHeader>
          <CardContent>
            <div className="space-y-2">
              <div className="flex justify-between text-sm">
                <span>Total robots:</span>
                <span className="font-medium">{stats.totalRobots}</span>
              </div>
              <div className="flex justify-between text-sm">
                <span>Robots per user:</span>
                <span className="font-medium">
                  {Math.round(stats.totalRobots / (stats.totalUsers || 1) * 10) / 10}
                </span>
              </div>
              <div className="flex justify-between text-sm">
                <span>Total alerts:</span>
                <span className="font-medium">{stats.totalAlerts}</span>
              </div>
            </div>
          </CardContent>
        </Card>
      </div>
    </div>
  );
}
