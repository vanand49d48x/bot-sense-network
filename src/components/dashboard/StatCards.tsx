
import { 
  Battery, 
  MapPin, 
  Bell, 
  ArrowRight
} from "lucide-react";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Robot } from "@/types/robot";
import { useEffect, useState } from "react";

interface StatCardsProps {
  robots: Robot[];
}

export function StatCards({ robots }: StatCardsProps) {
  // Use local state to ensure reactivity
  const [stats, setStats] = useState<{
    title: string;
    value: number;
    description: string;
    icon: any;
    className?: string;
  }[]>([]);

  // Update stats whenever robots prop changes
  useEffect(() => {
    const totalRobots = robots.length;
    const onlineRobots = robots.filter(r => r.status === 'online').length;
    const offlineRobots = robots.filter(r => r.status === 'offline').length;
    const warningRobots = robots.filter(r => r.status === 'warning').length;
    
    setStats([
      {
        title: "Total Robots",
        value: totalRobots,
        description: "Registered devices",
        icon: ArrowRight,
      },
      {
        title: "Online",
        value: onlineRobots,
        description: `${totalRobots ? Math.round((onlineRobots / totalRobots) * 100) : 0}% of fleet`,
        icon: Battery,
        className: "text-robot-online",
      },
      {
        title: "Warnings",
        value: warningRobots,
        description: "Require attention",
        icon: Bell,
        className: "text-robot-warning",
      },
      {
        title: "Offline",
        value: offlineRobots,
        description: "Not responding",
        icon: MapPin,
        className: "text-robot-offline",
      },
    ]);
  }, [robots]);

  return (
    <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-4">
      {stats.map((stat) => (
        <Card key={`${stat.title}-${stat.value}`} className="animate-fade-in">
          <CardHeader className="flex flex-row items-center justify-between pb-2">
            <CardTitle className="text-sm font-medium">{stat.title}</CardTitle>
            <stat.icon className={`h-4 w-4 ${stat.className || ""}`} />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{stat.value}</div>
            <p className="text-xs text-muted-foreground">{stat.description}</p>
          </CardContent>
        </Card>
      ))}
    </div>
  );
}
