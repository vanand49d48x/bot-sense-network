
import { 
  Battery, 
  MapPin, 
  Bell, 
  ArrowRight
} from "lucide-react";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Robot } from "@/types/robot";
import { Link } from "react-router-dom";
import { useState, useEffect } from "react";

interface StatCardsProps {
  robots: Robot[];
}

export function StatCards({ robots }: StatCardsProps) {
  // Create state for all stats
  const [stats, setStats] = useState<{
    totalRobots: number;
    onlineRobots: number;
    offlineRobots: number;
    warningRobots: number;
    initialized: boolean;
  }>({
    totalRobots: 0,
    onlineRobots: 0,
    offlineRobots: 0,
    warningRobots: 0,
    initialized: false
  });

  // Only update stats when robots prop changes
  useEffect(() => {
    // Skip empty arrays on initial render
    if (!robots || robots.length === 0) return;
    
    console.log("StatCards updating with robots:", robots.length);
    
    const totalRobots = robots.length;
    const onlineRobots = robots.filter(r => r.status === 'online').length;
    const offlineRobots = robots.filter(r => r.status === 'offline').length;
    const warningRobots = robots.filter(r => r.status === 'warning').length;

    setStats({
      totalRobots,
      onlineRobots,
      offlineRobots,
      warningRobots,
      initialized: true
    });
  }, [robots]);
  
  // If we haven't initialized with actual data yet, show zeros
  if (!stats.initialized && robots.length === 0) {
    return (
      <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-4">
        {[...Array(4)].map((_, i) => (
          <Card 
            key={i}
            className="animate-fade-in cursor-pointer hover:shadow-md transition-all"
          >
            <CardHeader className="flex flex-row items-center justify-between pb-2">
              <CardTitle className="text-sm font-medium">Loading...</CardTitle>
            </CardHeader>
            <CardContent>
              <div className="text-2xl font-bold">-</div>
              <p className="text-xs text-muted-foreground">Loading data</p>
            </CardContent>
          </Card>
        ))}
      </div>
    );
  }
  
  const statCards = [
    {
      title: "Total Robots",
      value: stats.totalRobots,
      description: "Registered devices",
      icon: ArrowRight,
      linkTo: "/dashboard",
    },
    {
      title: "Online",
      value: stats.onlineRobots,
      description: `${stats.totalRobots ? Math.round((stats.onlineRobots / stats.totalRobots) * 100) : 0}% of fleet`,
      icon: Battery,
      className: "text-robot-online",
      linkTo: "/fleet-status",
    },
    {
      title: "Warnings",
      value: stats.warningRobots,
      description: "Require attention",
      icon: Bell,
      className: "text-robot-warning",
      linkTo: "/alerts",
    },
    {
      title: "Offline",
      value: stats.offlineRobots,
      description: "Not responding",
      icon: MapPin,
      className: "text-robot-offline",
      linkTo: "/alerts",
    },
  ];

  return (
    <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-4">
      {statCards.map((stat) => (
        <Card 
          key={stat.title} 
          className="animate-fade-in cursor-pointer hover:shadow-md transition-all"
        >
          <Link to={stat.linkTo} className="block h-full">
            <CardHeader className="flex flex-row items-center justify-between pb-2">
              <CardTitle className="text-sm font-medium">{stat.title}</CardTitle>
              <stat.icon className={`h-4 w-4 ${stat.className || ""}`} />
            </CardHeader>
            <CardContent>
              <div className="text-2xl font-bold">{stat.value}</div>
              <p className="text-xs text-muted-foreground">{stat.description}</p>
            </CardContent>
          </Link>
        </Card>
      ))}
    </div>
  );
}
