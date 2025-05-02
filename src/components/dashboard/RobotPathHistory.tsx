
import React, { useState } from "react";
import { useQuery } from "@tanstack/react-query";
import { supabase } from "@/integrations/supabase/client";
import { format, subDays } from "date-fns";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { Label } from "@/components/ui/label";
import { Switch } from "@/components/ui/switch";
import { Map, History, Route } from "lucide-react";
import { Skeleton } from "@/components/ui/skeleton";
import { Alert, AlertDescription } from "@/components/ui/alert";
import { AlertCircle } from "lucide-react";
import { LeafletMap } from "./LeafletMap";
import { Robot } from "@/types/robot";

interface RobotPathHistoryProps {
  robot: Robot;
  retentionDays: number;
}

interface TelemetryLocation {
  id: string;
  created_at: string;
  location: {
    latitude: number;
    longitude: number;
  } | null;
}

type TimeRange = "24h" | "7d" | "30d" | "all";

export function RobotPathHistory({ robot, retentionDays }: RobotPathHistoryProps) {
  const [timeRange, setTimeRange] = useState<TimeRange>("24h");
  const [showCurrentLocation, setShowCurrentLocation] = useState(true);
  
  // Calculate date range based on selected time range
  const getDateRange = () => {
    const now = new Date();
    switch (timeRange) {
      case "24h":
        return subDays(now, 1).toISOString();
      case "7d":
        return subDays(now, 7).toISOString();
      case "30d":
        return subDays(now, 30).toISOString();
      case "all":
        return subDays(now, retentionDays).toISOString();
      default:
        return subDays(now, 1).toISOString();
    }
  };
  
  // Fetch location history data for the selected time range
  const { data: locationHistory, isLoading, error } = useQuery({
    queryKey: ['location-history', robot.id, timeRange, retentionDays],
    queryFn: async () => {
      const cutoffDate = getDateRange();
      
      const { data, error } = await supabase
        .from('telemetry')
        .select('id, created_at, location')
        .eq('robot_id', robot.id)
        .gte('created_at', cutoffDate)
        .order('created_at', { ascending: true })
        .not('location', 'is', null);

      if (error) throw new Error(error.message);
      
      return data as TelemetryLocation[];
    },
    enabled: !!robot.id,
  });

  // Format the historical path data for the LeafletMap component
  const historicalPaths = React.useMemo(() => {
    if (!locationHistory || locationHistory.length === 0) return [];
    
    // Filter out entries with null or invalid location data
    const validLocations = locationHistory.filter(
      item => item.location && 
      typeof item.location.latitude === 'number' && 
      typeof item.location.longitude === 'number'
    );
    
    if (validLocations.length <= 1) return [];
    
    return [{
      robotId: robot.id,
      path: validLocations.map(item => [item.location!.latitude, item.location!.longitude] as [number, number]),
      timestamp: validLocations[0].created_at,
      color: "#3b82f6"
    }];
  }, [locationHistory, robot.id]);

  // Prepare the robot data for the map component
  const mapRobots = React.useMemo(() => {
    return showCurrentLocation && robot.location ? [robot] : [];
  }, [robot, showCurrentLocation]);

  if (isLoading) {
    return (
      <div className="space-y-4">
        <div className="flex justify-between">
          <Skeleton className="h-10 w-48" />
          <Skeleton className="h-10 w-28" />
        </div>
        <Skeleton className="h-[300px] w-full" />
      </div>
    );
  }

  if (error) {
    return (
      <Alert variant="destructive">
        <AlertCircle className="h-4 w-4" />
        <AlertDescription>Failed to load location history: {String(error)}</AlertDescription>
      </Alert>
    );
  }

  return (
    <Card>
      <CardHeader className="pb-2">
        <div className="flex flex-col sm:flex-row sm:items-center justify-between gap-2">
          <CardTitle className="flex items-center gap-2">
            <Route className="h-5 w-5" />
            <span>Location History</span>
          </CardTitle>
          <div className="flex flex-wrap items-center gap-4">
            <div className="flex items-center gap-2">
              <Switch
                id="show-current"
                checked={showCurrentLocation}
                onCheckedChange={setShowCurrentLocation}
              />
              <Label htmlFor="show-current">Show Current</Label>
            </div>
            
            <div className="flex items-center gap-2">
              <Label htmlFor="time-range">History:</Label>
              <Select value={timeRange} onValueChange={(value) => setTimeRange(value as TimeRange)}>
                <SelectTrigger className="w-[100px]" id="time-range">
                  <SelectValue placeholder="Time range" />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="24h">24h</SelectItem>
                  <SelectItem value="7d">7 days</SelectItem>
                  <SelectItem value="30d">30 days</SelectItem>
                  <SelectItem value="all">All</SelectItem>
                </SelectContent>
              </Select>
            </div>
          </div>
        </div>
      </CardHeader>
      <CardContent>
        <div className="h-[300px] w-full overflow-hidden rounded-md border">
          {(historicalPaths.length > 0 || showCurrentLocation && robot.location) ? (
            <LeafletMap
              robots={mapRobots}
              height="300px"
              showTooltips={false}
              historicalPaths={historicalPaths}
            />
          ) : (
            <div className="flex flex-col items-center justify-center h-full text-muted-foreground">
              <Map className="h-12 w-12 mb-2 opacity-30" />
              <p>No location history available for the selected time period</p>
            </div>
          )}
        </div>
      </CardContent>
    </Card>
  );
}
