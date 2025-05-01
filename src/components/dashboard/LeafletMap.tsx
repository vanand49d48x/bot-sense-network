
import { useEffect, useState } from 'react';
import { MapContainer, TileLayer, Marker } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import { Robot } from "@/types/robot";
import L from 'leaflet';
import { HoverCard, HoverCardContent, HoverCardTrigger } from "@/components/ui/hover-card";

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
      status === 'online' ? 'bg-green-500' :
      status === 'warning' ? 'bg-yellow-500' : 'bg-red-500'
    } ${status === 'online' ? 'pulse-animation' : ''}" />`,
    iconSize: [16, 16],
    iconAnchor: [8, 8],
  });
};

interface LeafletMapProps {
  robots: Robot[];
  height?: string;
}

export function LeafletMap({ robots, height = "100%" }: LeafletMapProps) {
  const [isClient, setIsClient] = useState(false);
  const [hoveredRobotId, setHoveredRobotId] = useState<string | null>(null);
  
  // Create a map key that changes whenever robot positions change
  const mapKey = JSON.stringify(robots.map(r => 
    r.location ? `${r.id}-${r.location.latitude}-${r.location.longitude}-${r.status}` : r.id
  ));
  
  useEffect(() => {
    setIsClient(true);
    
    // Add the pulse animation CSS
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
      
      /* Custom styles for the robot hover tooltips */
      .robot-marker {
        position: relative;
        z-index: 1;
      }
      
      .robot-marker:hover {
        z-index: 1000;
      }
      
      /* Override Leaflet's default tooltip styles */
      .leaflet-tooltip {
        background-color: transparent !important;
        border: none !important;
        box-shadow: none !important;
        padding: 0 !important;
      }
    `;
    document.head.appendChild(style);

    return () => {
      document.head.removeChild(style);
    };
  }, []);

  // Default position (San Francisco)
  const defaultPosition: [number, number] = [37.7749, -122.4194];

  // Filter robots that have location data
  const robotsWithLocation = robots.filter(robot => robot.location !== undefined);
  
  // Calculate center based on available robots if we have locations
  const center = robotsWithLocation.length > 0 
    ? getMapCenter(robotsWithLocation)
    : defaultPosition;

  // Create custom marker elements with hover functionality
  const createCustomMarker = (robot: Robot) => {
    // Create a div icon with hover capabilities
    return L.divIcon({
      className: 'robot-marker',
      html: `
        <div 
          class="w-4 h-4 rounded-full ${
            robot.status === 'online' ? 'bg-green-500' :
            robot.status === 'warning' ? 'bg-yellow-500' : 'bg-red-500'
          } ${robot.status === 'online' ? 'pulse-animation' : ''}"
          data-robot-id="${robot.id}"
        />
      `,
      iconSize: [16, 16],
      iconAnchor: [8, 8],
    });
  };

  // Only render the map on the client-side to avoid SSR issues
  if (!isClient) {
    return <div style={{ height, width: "100%" }} className="bg-gray-100 animate-pulse rounded-md"></div>;
  }

  return (
    <>
      <MapContainer 
        key={mapKey} // This forces the map to re-render when robot data changes
        center={center} 
        zoom={robotsWithLocation.length > 1 ? 10 : 13} 
        style={{ height, width: "100%" }}
      >
        <TileLayer
          attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />
        {robotsWithLocation.map(robot => (
          <Marker
            key={`${robot.id}-${robot.lastHeartbeat}`}
            position={[
              robot.location!.latitude,
              robot.location!.longitude,
            ]}
            icon={createCustomMarker(robot)}
            eventHandlers={{
              mouseover: () => setHoveredRobotId(robot.id),
              mouseout: () => setHoveredRobotId(null),
            }}
          />
        ))}
      </MapContainer>
      
      {/* Floating tooltip that follows cursor */}
      {robotsWithLocation.map(robot => (
        <div key={robot.id} className="relative">
          <div style={{ display: hoveredRobotId === robot.id ? 'block' : 'none' }} 
               className="absolute z-50 pointer-events-none">
            <div className="bg-background/90 backdrop-blur-sm border rounded-md shadow-md p-3 max-w-xs">
              <h4 className="font-semibold">{robot.name}</h4>
              <div className="grid grid-cols-2 gap-x-2 text-sm mt-1">
                <div className="text-muted-foreground">Battery:</div>
                <div>{robot.batteryLevel}%</div>
                
                <div className="text-muted-foreground">Temp:</div>
                <div>{robot.temperature}°C</div>
                
                <div className="text-muted-foreground">Status:</div>
                <div className={`
                  ${robot.status === 'online' ? 'text-green-500' :
                    robot.status === 'warning' ? 'text-yellow-500' : 'text-red-500'}
                `}>
                  {robot.status}
                </div>
              </div>
            </div>
          </div>
        </div>
      ))}
    </>
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
