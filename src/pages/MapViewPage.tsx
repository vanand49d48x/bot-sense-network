
import { MainLayout } from "@/components/layout/MainLayout";
import { useRobots } from "@/hooks/useRobots";
import { Robot } from "@/types/robot";
import { mapSupabaseRobotToAppRobot } from "@/utils/robotMapper";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { MapPin, AlertTriangle, Battery, Info } from "lucide-react";
import { useEffect, useState } from "react";
import { 
  HoverCard,
  HoverCardContent,
  HoverCardTrigger,
} from "@/components/ui/hover-card";
import { LeafletMap } from "@/components/dashboard/LeafletMap";
import { 
  Tooltip,
  TooltipContent,
  TooltipTrigger,
} from "@/components/ui/tooltip";

const MapViewPage = () => {
  const { robots: supabaseRobots, loading } = useRobots();
  const [robots, setRobots] = useState<Robot[]>([]);
  
  // Map Supabase robots to application Robot type
  useEffect(() => {
    setRobots(supabaseRobots.map(mapSupabaseRobotToAppRobot));
  }, [supabaseRobots]);

  // Filter robots that have location data
  const robotsWithLocation = robots.filter(robot => robot.location !== undefined);
  
  if (loading) {
    return (
      <MainLayout>
        <div className="flex items-center justify-center min-h-[400px]">
          <div className="text-center">
            <div className="animate-pulse-slow">Loading map data...</div>
          </div>
        </div>
      </MainLayout>
    );
  }

  return (
    <MainLayout>
      <div className="space-y-6">
        <div className="flex items-center justify-between">
          <div>
            <h1 className="text-3xl font-bold tracking-tight">Robot Map</h1>
            <p className="text-muted-foreground mt-1">
              View and monitor your robot fleet's locations in real-time
            </p>
          </div>
          <Tooltip>
            <TooltipTrigger asChild>
              <div className="rounded-full bg-muted p-2 cursor-help">
                <Info className="h-5 w-5 text-muted-foreground" />
              </div>
            </TooltipTrigger>
            <TooltipContent>
              <p>Hover over robots on the map for more information</p>
            </TooltipContent>
          </Tooltip>
        </div>
        
        {robotsWithLocation.length > 0 ? (
          <Card className="animate-fade-in">
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                <MapPin className="h-5 w-5" />
                <span>Live Robot Locations</span>
              </CardTitle>
            </CardHeader>
            <CardContent>
              <div className="h-[500px] rounded-md overflow-hidden mb-4">
                <LeafletMap robots={robots} height="100%" showTooltips={true} />
              </div>
              <div className="grid grid-cols-1 md:grid-cols-3 gap-4 mt-6">
                {robotsWithLocation.map(robot => (
                  <Card key={robot.id} className="bg-muted/40">
                    <CardContent className="p-4">
                      <div className="flex items-center justify-between">
                        <HoverCard>
                          <HoverCardTrigger asChild>
                            <div className="flex items-center gap-2 cursor-pointer">
                              <div className={`w-3 h-3 rounded-full ${
                                robot.status === 'online' ? 'bg-green-500 pulse-animation' :
                                robot.status === 'warning' ? 'bg-yellow-500' : 'bg-red-500'
                              }`} />
                              <span className="font-medium">{robot.name}</span>
                            </div>
                          </HoverCardTrigger>
                          <HoverCardContent className="w-80 z-50">
                            <div className="space-y-2">
                              <h4 className="font-semibold">{robot.name}</h4>
                              <div className="grid grid-cols-2 gap-2 text-sm">
                                <div>Model:</div>
                                <div>{robot.model}</div>
                                <div>Status:</div>
                                <div className={`
                                  ${robot.status === 'online' ? 'text-green-500' :
                                    robot.status === 'warning' ? 'text-yellow-500' : 'text-red-500'}
                                `}>{robot.status}</div>
                                <div>Battery:</div>
                                <div className="flex items-center gap-1">
                                  <Battery className="h-4 w-4" />
                                  {robot.batteryLevel}%
                                </div>
                                <div>Temperature:</div>
                                <div>{robot.temperature}°C</div>
                                <div>IP Address:</div>
                                <div>{robot.ipAddress}</div>
                                <div>Last Seen:</div>
                                <div>{new Date(robot.lastHeartbeat).toLocaleTimeString()}</div>
                              </div>
                            </div>
                          </HoverCardContent>
                        </HoverCard>
                        <div className="flex items-center gap-2">
                          <div className="text-sm text-muted-foreground">
                            {robot.location ? 
                              `${robot.location.latitude.toFixed(4)}, ${robot.location.longitude.toFixed(4)}` : 
                              'No location'
                            }
                          </div>
                        </div>
                      </div>
                    </CardContent>
                  </Card>
                ))}
              </div>
            </CardContent>
          </Card>
        ) : (
          <div className="rounded-lg border border-dashed p-12 text-center">
            <MapPin className="h-12 w-12 mx-auto mb-4 text-muted-foreground" />
            <h3 className="text-lg font-medium mb-2">No robot location data available</h3>
            <p className="text-muted-foreground">
              None of your robots are currently reporting location data
            </p>
          </div>
        )}
      </div>
    </MainLayout>
  );
};

export default MapViewPage;
