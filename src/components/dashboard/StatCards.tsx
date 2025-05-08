
import { 
  Battery, 
  MapPin, 
  Bell, 
  ArrowRight
} from "lucide-react";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Robot } from "@/types/robot";
import { Link } from "react-router-dom";

interface StatCardsProps {
  robots: Robot[];
}

export function StatCards({ robots }: StatCardsProps) {
  const totalRobots = robots.length;
  const onlineRobots = robots.filter(r => r.status === 'online').length;
  const offlineRobots = robots.filter(r => r.status === 'offline').length;
  const warningRobots = robots.filter(r => r.status === 'warning').length;
  
  const stats = [
    {
      title: "Total Robots",
      value: totalRobots,
      description: "Registered devices",
      icon: ArrowRight,
      linkTo: "/dashboard",
    },
    {
      title: "Online",
      value: onlineRobots,
      description: `${totalRobots ? Math.round((onlineRobots / totalRobots) * 100) : 0}% of fleet`,
      icon: Battery,
      className: "text-robot-online",
      linkTo: "/fleet-status",
    },
    {
      title: "Warnings",
      value: warningRobots,
      description: "Require attention",
      icon: Bell,
      className: "text-robot-warning",
      linkTo: "/alerts",
    },
    {
      title: "Offline",
      value: offlineRobots,
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
