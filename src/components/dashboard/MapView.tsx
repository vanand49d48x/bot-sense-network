
import { useState, useEffect } from "react";
import { Robot } from "@/types/robot";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";

interface MapViewProps {
  robots: Robot[];
}

export function MapView({ robots }: MapViewProps) {
  const [mapLoaded, setMapLoaded] = useState(false);

  useEffect(() => {
    // In a real app, this would initialize a map library like Leaflet
    const timer = setTimeout(() => setMapLoaded(true), 1000);
    return () => clearTimeout(timer);
  }, []);

  const robotsWithLocation = robots.filter(robot => robot.location);

  if (robotsWithLocation.length === 0) {
    return null;
  }

  return (
    <Card className="mt-6 animate-fade-in">
      <CardHeader>
        <CardTitle>Robot Locations</CardTitle>
      </CardHeader>
      <CardContent>
        {!mapLoaded ? (
          <div className="h-64 flex items-center justify-center">
            <div className="text-center">
              <div className="animate-pulse-slow">Loading map...</div>
            </div>
          </div>
        ) : (
          <div className="relative h-64 bg-muted/30 rounded-md overflow-hidden">
            <div className="absolute inset-0 flex items-center justify-center opacity-20 text-7xl font-bold">MAP VIEW</div>
            {/* Map placeholder - in a real implementation, this would be a leaflet.js map */}
            {robotsWithLocation.map((robot) => (
              <div 
                key={robot.id}
                className={`absolute w-3 h-3 rounded-full transform -translate-x-1/2 -translate-y-1/2 ${
                  robot.status === 'online' ? 'bg-robot-online' :
                  robot.status === 'warning' ? 'bg-robot-warning' : 'bg-robot-offline'
                }`}
                style={{
                  // This is just a placeholder positioning - in a real app, we'd convert GPS to pixels
                  left: `${((robot.location?.lng || 0) + 122.42) * 100}%`,
                  top: `${(37.78 - (robot.location?.lat || 0)) * 100}%`
                }}
              >
                {robot.status === 'online' && (
                  <span className="absolute inset-0 rounded-full bg-robot-online animate-ping-slow opacity-75"></span>
                )}
                <div className="absolute top-full left-1/2 transform -translate-x-1/2 mt-1 bg-background px-2 py-1 rounded text-xs whitespace-nowrap">
                  {robot.name}
                </div>
              </div>
            ))}
          </div>
        )}
        <div className="mt-4 text-xs text-muted-foreground">
          Showing {robotsWithLocation.length} robots with location data
        </div>
      </CardContent>
    </Card>
  );
}
