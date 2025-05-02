
import { MainLayout } from "@/components/layout/MainLayout";
import { useRobots } from "@/hooks/useRobots";
import { Robot } from "@/types/robot";
import { mapSupabaseRobotToAppRobot } from "@/utils/robotMapper";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { MapPin, AlertTriangle, Battery, Info, Map, Plus } from "lucide-react";
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
import { Button } from "@/components/ui/button";
import {
  Tabs,
  TabsContent,
  TabsList,
  TabsTrigger,
} from "@/components/ui/tabs";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Switch } from "@/components/ui/switch";

// Sample data for historical paths
const sampleHistoricalPaths = [
  {
    robotId: "1",
    path: [
      [37.7749, -122.4194],
      [37.7750, -122.4180],
      [37.7753, -122.4170],
      [37.7756, -122.4168],
    ] as [number, number][],
    timestamp: new Date().toISOString(),
    color: "#3b82f6"
  },
  {
    robotId: "2",
    path: [
      [37.7730, -122.4190],
      [37.7725, -122.4185],
      [37.7720, -122.4175],
      [37.7715, -122.4165],
    ] as [number, number][],
    timestamp: new Date(Date.now() - 3600000).toISOString(),
    color: "#10b981"
  }
];

// Sample data for geofence zones
const sampleGeofenceZones = [
  {
    id: "1",
    name: "Safe Zone",
    center: [37.7749, -122.4194] as [number, number],
    radius: 500,
    color: "#10b981"
  },
  {
    id: "2",
    name: "Caution Zone",
    center: [37.7720, -122.4170] as [number, number],
    radius: 300,
    color: "#f59e0b"
  }
];

const MapViewPage = () => {
  const { robots: supabaseRobots, loading } = useRobots();
  const [robots, setRobots] = useState<Robot[]>([]);
  const [showGeofencing, setShowGeofencing] = useState(true);
  const [showHistoricalPaths, setShowHistoricalPaths] = useState(true);
  const [historicalPaths, setHistoricalPaths] = useState(sampleHistoricalPaths);
  const [geofenceZones, setGeofenceZones] = useState(sampleGeofenceZones);
  const [mapMode, setMapMode] = useState<'live' | 'paths'>('live');
  const [selectedRobot, setSelectedRobot] = useState<string | null>(null);
  const [isAddZoneOpen, setIsAddZoneOpen] = useState(false);
  const [newZone, setNewZone] = useState({
    name: '',
    radius: 200,
    color: '#10b981'
  });
  
  // Map Supabase robots to application Robot type
  useEffect(() => {
    setRobots(supabaseRobots.map(mapSupabaseRobotToAppRobot));
  }, [supabaseRobots]);

  // Filter robots that have location data
  const robotsWithLocation = robots.filter(robot => robot.location !== undefined);

  // Function to handle adding a new geofence zone
  const handleAddZone = () => {
    if (!newZone.name) return;
    
    // Use the center of the map or first robot's location as default
    const zoneCenter = robotsWithLocation.length > 0
      ? [robotsWithLocation[0].location!.latitude, robotsWithLocation[0].location!.longitude] as [number, number]
      : [37.7749, -122.4194] as [number, number];
    
    const newGeofenceZone = {
      id: Date.now().toString(),
      name: newZone.name,
      center: zoneCenter,
      radius: newZone.radius,
      color: newZone.color
    };
    
    setGeofenceZones([...geofenceZones, newGeofenceZone]);
    setNewZone({ name: '', radius: 200, color: '#10b981' });
    setIsAddZoneOpen(false);
  };

  // Filter robots by selected robot
  const filteredRobots = selectedRobot && selectedRobot !== 'all'
    ? robots.filter(robot => robot.id === selectedRobot)
    : robots;

  // Filter historical paths by selected robot
  const filteredPaths = selectedRobot && selectedRobot !== 'all'
    ? historicalPaths.filter(path => path.robotId === selectedRobot)
    : historicalPaths;

  // Displayed geofence zones based on toggle
  const displayedGeofenceZones = showGeofencing ? geofenceZones : [];
  
  // Displayed historical paths based on toggle
  const displayedHistoricalPaths = showHistoricalPaths ? filteredPaths : [];
  
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
              <div className="flex justify-between items-center">
                <CardTitle className="flex items-center gap-2">
                  <MapPin className="h-5 w-5" />
                  <span>Live Robot Locations</span>
                </CardTitle>
                <Tabs defaultValue="map-options" className="w-[400px]">
                  <TabsList className="grid w-full grid-cols-2">
                    <TabsTrigger value="map-options">Map Options</TabsTrigger>
                    <TabsTrigger value="geofencing">Geofencing</TabsTrigger>
                  </TabsList>
                  <TabsContent value="map-options" className="space-y-4">
                    <div className="grid grid-cols-2 gap-4">
                      <div className="flex items-center space-x-2">
                        <Switch 
                          id="show-historical" 
                          checked={showHistoricalPaths} 
                          onCheckedChange={setShowHistoricalPaths} 
                        />
                        <Label htmlFor="show-historical">Show Historical Paths</Label>
                      </div>
                      <div className="flex items-center space-x-2">
                        <Switch 
                          id="show-zones" 
                          checked={showGeofencing} 
                          onCheckedChange={setShowGeofencing} 
                        />
                        <Label htmlFor="show-zones">Show Geofence Zones</Label>
                      </div>
                    </div>
                    {showHistoricalPaths && (
                      <div className="space-y-2">
                        <Label htmlFor="robot-filter">Filter by Robot</Label>
                        <Select
                          value={selectedRobot || undefined}
                          onValueChange={(value) => setSelectedRobot(value)}
                        >
                          <SelectTrigger id="robot-filter" className="w-full">
                            <SelectValue placeholder="All Robots" />
                          </SelectTrigger>
                          <SelectContent>
                            <SelectItem value="all">All Robots</SelectItem>
                            {robots.map(robot => (
                              <SelectItem key={robot.id} value={robot.id}>{robot.name}</SelectItem>
                            ))}
                          </SelectContent>
                        </Select>
                      </div>
                    )}
                  </TabsContent>
                  <TabsContent value="geofencing" className="space-y-4">
                    <div className="flex justify-between items-center">
                      <h3 className="text-sm font-medium">Geofence Zones</h3>
                      <Dialog open={isAddZoneOpen} onOpenChange={setIsAddZoneOpen}>
                        <DialogTrigger asChild>
                          <Button size="sm" variant="outline">
                            <Plus className="mr-1 h-4 w-4" />
                            Add Zone
                          </Button>
                        </DialogTrigger>
                        <DialogContent>
                          <DialogHeader>
                            <DialogTitle>Create Geofence Zone</DialogTitle>
                            <DialogDescription>
                              Define a new geofence zone for your robots.
                            </DialogDescription>
                          </DialogHeader>
                          <div className="grid gap-4 py-4">
                            <div className="grid grid-cols-4 items-center gap-4">
                              <Label htmlFor="zone-name" className="text-right">
                                Name
                              </Label>
                              <Input
                                id="zone-name"
                                value={newZone.name}
                                onChange={(e) => setNewZone({...newZone, name: e.target.value})}
                                className="col-span-3"
                              />
                            </div>
                            <div className="grid grid-cols-4 items-center gap-4">
                              <Label htmlFor="zone-radius" className="text-right">
                                Radius (m)
                              </Label>
                              <Input
                                id="zone-radius"
                                type="number"
                                value={newZone.radius}
                                onChange={(e) => setNewZone({...newZone, radius: parseInt(e.target.value)})}
                                className="col-span-3"
                              />
                            </div>
                            <div className="grid grid-cols-4 items-center gap-4">
                              <Label htmlFor="zone-color" className="text-right">
                                Color
                              </Label>
                              <div className="col-span-3 flex items-center gap-2">
                                <input
                                  type="color"
                                  value={newZone.color}
                                  onChange={(e) => setNewZone({...newZone, color: e.target.value})}
                                  className="w-10 h-10 rounded border"
                                />
                                <Input
                                  id="zone-color"
                                  value={newZone.color}
                                  onChange={(e) => setNewZone({...newZone, color: e.target.value})}
                                  className="flex-1"
                                />
                              </div>
                            </div>
                          </div>
                          <DialogFooter>
                            <Button type="submit" onClick={handleAddZone}>Create Zone</Button>
                          </DialogFooter>
                        </DialogContent>
                      </Dialog>
                    </div>
                    <div className="space-y-2 max-h-36 overflow-y-auto">
                      {geofenceZones.map((zone) => (
                        <div 
                          key={zone.id} 
                          className="flex items-center justify-between py-2 px-3 border rounded-md"
                        >
                          <div className="flex items-center gap-2">
                            <div 
                              className="w-3 h-3 rounded-full" 
                              style={{ backgroundColor: zone.color }}
                            ></div>
                            <span className="font-medium text-sm">{zone.name}</span>
                          </div>
                          <span className="text-xs text-muted-foreground">
                            {zone.radius}m radius
                          </span>
                        </div>
                      ))}
                    </div>
                  </TabsContent>
                </Tabs>
              </div>
            </CardHeader>
            <CardContent>
              <div className="h-[500px] rounded-md overflow-hidden mb-4">
                <LeafletMap 
                  robots={filteredRobots}
                  height="100%"
                  showTooltips={true}
                  historicalPaths={displayedHistoricalPaths}
                  geofenceZones={displayedGeofenceZones}
                />
              </div>
              <div className="grid grid-cols-1 md:grid-cols-3 gap-4 mt-6">
                {filteredRobots.filter(robot => robot.location !== undefined).map(robot => (
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
