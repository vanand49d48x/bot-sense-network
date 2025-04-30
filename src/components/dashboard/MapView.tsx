
import { Robot } from "@/types/robot";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { LeafletMap } from "./LeafletMap";

interface MapViewProps {
  robots: Robot[];
}

export function MapView({ robots }: MapViewProps) {
  // Generate a robots map key based on their locations and heartbeats
  const robotsKey = robots.filter(r => r.location).map(r => 
    `${r.id}-${r.location?.latitude}-${r.location?.longitude}-${r.lastHeartbeat}`
  ).join('|');

  // Filter robots that have location data
  const robotsWithLocation = robots.filter(robot => robot.location !== undefined);

  if (robotsWithLocation.length === 0) {
    return null;
  }

  return (
    <Card className="mt-6 animate-fade-in">
      <CardHeader>
        <CardTitle>Robot Locations</CardTitle>
      </CardHeader>
      <CardContent>
        <div className="h-64 rounded-md overflow-hidden">
          <LeafletMap 
            robots={robots} 
            height="100%" 
            key={robotsKey} // This ensures the map re-renders with new data
          />
        </div>
        <div className="mt-4 text-xs text-muted-foreground">
          Showing {robotsWithLocation.length} robots with location data
        </div>
      </CardContent>
    </Card>
  );
}
