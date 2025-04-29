
import { useEffect, useState } from 'react';
import { MapContainer, TileLayer, Marker, Popup } from 'react-leaflet';
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

  // Only render the map on the client-side to avoid SSR issues
  if (!isClient) {
    return <div style={{ height, width: "100%" }} className="bg-gray-100 animate-pulse rounded-md"></div>;
  }

  return (
    <MapContainer 
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
                ${robot.status === 'online' ? 'text-green-500' :
                  robot.status === 'warning' ? 'text-yellow-500' : 'text-red-500'}
              `}>{robot.status}</span>
            </div>
          </Popup>
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
