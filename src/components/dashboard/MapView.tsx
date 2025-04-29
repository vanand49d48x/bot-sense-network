
import { MapContainer, TileLayer, Marker, Popup } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import { Robot } from "@/types/robot";
import L from 'leaflet';
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";

// Fix Leaflet's missing marker icons
delete (L.Icon.Default.prototype as any)._getIconUrl;
L.Icon.Default.mergeOptions({
  iconRetinaUrl: "https://unpkg.com/leaflet@1.9.3/dist/images/marker-icon-2x.png",
  iconUrl: "https://unpkg.com/leaflet@1.9.3/dist/images/marker-icon.png",
  shadowUrl: "https://unpkg.com/leaflet@1.9.3/dist/images/marker-shadow.png",
});

// Custom marker icons for different robot statuses
const createStatusIcon = (status: 'online' | 'offline' | 'warning') => {
  return L.divIcon({
    className: 'custom-marker',
    html: `<div class="w-4 h-4 rounded-full ${
      status === 'online' ? 'bg-robot-online' :
      status === 'warning' ? 'bg-robot-warning' : 'bg-robot-offline'
    } ${status === 'online' ? 'pulse-animation' : ''}" />`,
    iconSize: [16, 16],
    iconAnchor: [8, 8],
  });
};

interface MapViewProps {
  robots: Robot[];
}

export function MapView({ robots }: MapViewProps) {
  // Default position (San Francisco)
  const defaultPosition: [number, number] = [37.7749, -122.4194];

  // Filter robots that have location data
  const robotsWithLocation = robots.filter(robot => robot.location !== undefined);

  if (robotsWithLocation.length === 0) {
    return null;
  }

  // Calculate center based on available robots if we have locations
  const center = robotsWithLocation.length > 0 
    ? getMapCenter(robotsWithLocation)
    : defaultPosition;

  return (
    <Card className="mt-6 animate-fade-in">
      <CardHeader>
        <CardTitle>Robot Locations</CardTitle>
      </CardHeader>
      <CardContent>
        <div className="h-64 rounded-md overflow-hidden">
          <MapContainer 
            center={center} 
            zoom={robotsWithLocation.length > 1 ? 10 : 13} 
            style={{ height: "100%", width: "100%" }}
          >
            <TileLayer
              attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
              url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
            />
            {robotsWithLocation.map(robot => (
              <Marker
                key={robot.id}
                position={[
                  robot.location!.latitude,
                  robot.location!.longitude,
                ]}
                icon={createStatusIcon(robot.status)}
              >
                <Popup>
                  <div className="text-sm">
                    <strong>{robot.name}</strong><br />
                    Battery: {robot.batteryLevel}%<br />
                    Temp: {robot.temperature}°C<br />
                    Status: <span className={`
                      ${robot.status === 'online' ? 'text-robot-online' :
                        robot.status === 'warning' ? 'text-robot-warning' : 'text-robot-offline'}
                    `}>{robot.status}</span>
                  </div>
                </Popup>
              </Marker>
            ))}
          </MapContainer>
        </div>
        <div className="mt-4 text-xs text-muted-foreground">
          Showing {robotsWithLocation.length} robots with location data
        </div>
      </CardContent>
    </Card>
  );
}

// Helper function to calculate the center of the map based on robot positions
function getMapCenter(robots: Robot[]): [number, number] {
  if (robots.length === 0) return [37.7749, -122.4194]; // Default to SF
  
  let sumLat = 0;
  let sumLng = 0;
  
  robots.forEach(robot => {
    if (robot.location) {
      sumLat += robot.location.latitude;
      sumLng += robot.location.longitude;
    }
  });
  
  return [sumLat / robots.length, sumLng / robots.length];
}

// Add the pulse animation
const style = document.createElement('style');
style.innerHTML = `
  .pulse-animation {
    animation: pulse 1.5s infinite;
  }
  @keyframes pulse {
    0% {
      box-shadow: 0 0 0 0 rgba(16, 185, 129, 0.7);
    }
    70% {
      box-shadow: 0 0 0 10px rgba(16, 185, 129, 0);
    }
    100% {
      box-shadow: 0 0 0 0 rgba(16, 185, 129, 0);
    }
  }
`;
document.head.appendChild(style);
