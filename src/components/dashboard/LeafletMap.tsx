
import { useEffect, useState, useRef } from 'react';
import { MapContainer, TileLayer, Marker, Tooltip } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import { Robot } from "@/types/robot";
import L from 'leaflet';

// Fix Leaflet's missing marker icons
delete (L.Icon.Default.prototype as any)._getIconUrl;
L.Icon.Default.mergeOptions({
  iconRetinaUrl: "https://unpkg.com/leaflet@1.9.3/dist/images/marker-icon-2x.png",
  iconUrl: "https://unpkg.com/leaflet@1.9.3/dist/images/marker-icon.png",
  shadowUrl: "https://unpkg.com/leaflet@1.9.3/dist/images/marker-shadow.png",
});

interface LeafletMapProps {
  robots: Robot[];
  height?: string;
}

export function LeafletMap({ robots, height = "100%" }: LeafletMapProps) {
  const [isClient, setIsClient] = useState(false);
  
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
      
      /* Custom robot marker styles */
      .robot-marker {
        display: flex;
        align-items: center;
        justify-content: center;
        width: 16px !important;
        height: 16px !important;
        border-radius: 50%;
        z-index: 1000;
      }
      
      .robot-marker.online {
        background-color: rgb(34, 197, 94);
      }
      
      .robot-marker.warning {
        background-color: rgb(234, 179, 8);
      }
      
      .robot-marker.offline {
        background-color: rgb(239, 68, 68);
      }
      
      .robot-marker.online {
        animation: pulse 1.5s infinite;
      }
      
      /* Custom tooltip styles */
      .leaflet-tooltip.robot-tooltip {
        background-color: rgba(255, 255, 255, 0.95);
        border: 1px solid #e2e8f0;
        border-radius: 6px;
        box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
        padding: 8px;
        font-size: 12px;
        min-width: 150px;
      }
      
      .leaflet-tooltip-top.robot-tooltip:before {
        border-top-color: rgba(255, 255, 255, 0.95);
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

  // Create custom marker icons based on robot status
  const createCustomMarkerIcon = (status: 'online' | 'offline' | 'warning') => {
    return L.divIcon({
      className: `robot-marker ${status}`,
      iconSize: [16, 16],
      iconAnchor: [8, 8],
    });
  };

  // Only render the map on the client-side to avoid SSR issues
  if (!isClient) {
    return <div style={{ height, width: "100%" }} className="bg-gray-100 animate-pulse rounded-md"></div>;
  }

  return (
    <MapContainer 
      key={mapKey}
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
          icon={createCustomMarkerIcon(robot.status)}
        >
          <Tooltip 
            direction="top" 
            offset={[0, -8]} 
            opacity={1.0} 
            permanent={false} 
            interactive={true}
            className="robot-tooltip"
          >
            <div>
              <h4 className="font-semibold">{robot.name}</h4>
              <div className="grid grid-cols-2 gap-x-2 text-xs mt-1">
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
          </Tooltip>
        </Marker>
      ))}
    </MapContainer>
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
