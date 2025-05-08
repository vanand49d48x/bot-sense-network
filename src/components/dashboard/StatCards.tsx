
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
  // Create local state to ensure stable rendering
  const [robotStats, setRobotStats] = useState({
    totalRobots: 0,
    onlineRobots: 0,
    offlineRobots: 0,
    warningRobots: 0
  });

  // Update stats when robots array changes
  useEffect(() => {
    if (robots.length > 0) {
      const onlineCount = robots.filter(r => r.status === 'online').length;
      const offlineCount = robots.filter(r => r.status === 'offline').length;
      const warningCount = robots.filter(r => r.status === 'warning').length;
      
      setRobotStats({
        totalRobots: robots.length,
        onlineRobots: onlineCount,
        offlineRobots: offlineCount,
        warningRobots: warningCount
      });
    }
  }, [robots]);
  
  const stats = [
    {
      title: "Total Robots",
      value: robotStats.totalRobots,
      description: "Registered devices",
      icon: ArrowRight,
      linkTo: "/dashboard",
    },
    {
      title: "Online",
      value: robotStats.onlineRobots,
      description: `${robotStats.totalRobots ? Math.round((robotStats.onlineRobots / robotStats.totalRobots) * 100) : 0}% of fleet`,
      icon: Battery,
      className: "text-robot-online",
      linkTo: "/fleet-status",
    },
    {
      title: "Warnings",
      value: robotStats.warningRobots,
      description: "Require attention",
      icon: Bell,
      className: "text-robot-warning",
      linkTo: "/alerts",
    },
    {
      title: "Offline",
      value: robotStats.offlineRobots,
      description: "Not responding",
      icon: MapPin,
      className: "text-robot-offline",
      linkTo: "/alerts",
    },
  ];

  return (
    <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-4">
      {stats.map((stat) => (
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
