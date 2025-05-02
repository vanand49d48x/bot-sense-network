
import { Robot } from "@/types/robot";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { LeafletMap } from "./LeafletMap";

interface MapViewProps {
  robots: Robot[];
  selectedRobotIds?: string[];
}

export function MapView({ robots, selectedRobotIds }: MapViewProps) {
  // Filter robots that have location data
  const robotsWithLocation = robots.filter(robot => robot.location !== undefined);
  
  // Apply filter by selected robot ids if provided
  const filteredRobots = selectedRobotIds && selectedRobotIds.length > 0 && !selectedRobotIds.includes('all')
    ? robotsWithLocation.filter(robot => selectedRobotIds.includes(robot.id))
    : robotsWithLocation;

  if (filteredRobots.length === 0) {
    return null;
  }

  return (
    <Card className="mt-6 animate-fade-in">
      <CardHeader>
        <CardTitle>Robot Locations</CardTitle>
      </CardHeader>
      <CardContent>
        <div className="h-64 rounded-md overflow-hidden">
          <LeafletMap robots={filteredRobots} height="100%" showTooltips={true} />
        </div>
        <div className="mt-4 text-xs text-muted-foreground">
          Showing {filteredRobots.length} robots with location data
        </div>
      </CardContent>
    </Card>
  );
}
