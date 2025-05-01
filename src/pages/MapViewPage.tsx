
import { MainLayout } from "@/components/layout/MainLayout";
import { LeafletMap } from "@/components/dashboard/LeafletMap";
import { useRobots } from "@/hooks/useRobots";
import { mapSupabaseRobotToAppRobot } from "@/utils/robotMapper";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";

export default function MapViewPage() {
  const { robots: supabaseRobots, loading } = useRobots();
  
  // Map Supabase robots to application Robot type
  const robots = supabaseRobots.map(mapSupabaseRobotToAppRobot);
  
  // Generate a robots map key based on their locations and heartbeats
  const robotsKey = robots
    .filter(r => r.location)
    .map(r => `${r.id}-${r.location?.latitude}-${r.location?.longitude}-${r.lastHeartbeat}`)
    .join('|');
  
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
      <div className="container py-6">
        <h1 className="text-3xl font-bold tracking-tight mb-6">Map View</h1>
        
        <Card>
          <CardHeader>
            <CardTitle>Robot Locations</CardTitle>
          </CardHeader>
          <CardContent>
            <div className="h-[70vh] rounded-md overflow-hidden">
              <LeafletMap 
                robots={robots} 
                height="100%" 
                key={robotsKey}
              />
            </div>
            <div className="mt-4 text-xs text-muted-foreground">
              {robotsWithLocation.length > 0 
                ? `Showing ${robotsWithLocation.length} robots with location data`
                : "No robots with location data available"
              }
            </div>
          </CardContent>
        </Card>

        {robotsWithLocation.length === 0 && (
          <div className="text-center p-6 border border-dashed rounded-lg mt-6">
            <h3 className="text-lg font-medium mb-2">No location data available</h3>
            <p className="text-muted-foreground">
              Connect robots with GPS capabilities to view them on the map
            </p>
          </div>
        )}
      </div>
    </MainLayout>
  );
}
