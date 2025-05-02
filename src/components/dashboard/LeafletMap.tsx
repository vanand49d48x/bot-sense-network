
import { useEffect, useState } from 'react';
import { MapContainer, TileLayer, Marker, Popup, Tooltip, Polyline, Circle, LayerGroup, LayersControl } from 'react-leaflet';
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

interface GeofenceZone {
  id: string;
  name: string;
  center: [number, number];
  radius: number; // in meters
  color: string;
}

interface HistoricalPath {
  robotId: string;
  path: [number, number][];
  timestamp: string;
  color: string;
}

interface LeafletMapProps {
  robots: Robot[];
  height?: string;
  showTooltips?: boolean;
  historicalPaths?: HistoricalPath[];
  geofenceZones?: GeofenceZone[];
  buildingOverlays?: Array<{
    name: string;
    bounds: [[number, number], [number, number]];
    imageUrl: string;
  }>;
}

export function LeafletMap({ 
  robots, 
  height = "100%", 
  showTooltips = false,
  historicalPaths = [],
  geofenceZones = [],
  buildingOverlays = []
}: LeafletMapProps) {
  const [isClient, setIsClient] = useState(false);
  
  useEffect(() => {
    setIsClient(true);
    
    // Add the pulse animation CSS and enhanced tooltip styles
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
      .leaflet-tooltip {
        background-color: rgba(26, 31, 44, 0.95);
        border: 1px solid rgba(255, 255, 255, 0.2);
        border-radius: 6px;
        padding: 10px 12px;
        font-size: 12px;
        box-shadow: 0 4px 12px rgba(0, 0, 0, 0.2);
        color: #FFFFFF;
        max-width: 220px;
        transition: all 0.2s ease;
      }
      .leaflet-tooltip-top:before {
        border-top-color: rgba(26, 31, 44, 0.95);
      }
      .tooltip-title {
        font-weight: 600;
        font-size: 14px;
        margin-bottom: 6px;
        border-bottom: 1px solid rgba(255, 255, 255, 0.2);
        padding-bottom: 4px;
      }
      .tooltip-grid {
        display: grid;
        grid-template-columns: 1fr 1fr;
        gap: 4px 8px;
      }
      .tooltip-label {
        color: rgba(255, 255, 255, 0.7);
        font-size: 11px;
      }
      .tooltip-value {
        font-weight: 500;
        text-align: right;
      }
      .tooltip-value-online {
        color: #10b981;
      }
      .tooltip-value-warning {
        color: #f59e0b;
      }
      .tooltip-value-offline {
        color: #ef4444;
      }
      .geofence-label {
        background: transparent;
        border: none;
        box-shadow: none;
        color: white;
        font-weight: bold;
        text-shadow: 0px 0px 3px rgba(0, 0, 0, 0.75);
      }
      .path-arrow {
        font-size: 20px;
        color: #3b82f6;
        text-shadow: 0px 0px 2px rgba(255, 255, 255, 0.8);
        pointer-events: none;
      }
      .path-arrow-container {
        display: flex;
        align-items: center;
        justify-content: center;
        width: 20px;
        height: 20px;
        border-radius: 50%;
        background-color: rgba(255, 255, 255, 0.7);
        border: 1px solid #3b82f6;
      }
      @media (max-width: 640px) {
        .leaflet-control-container .leaflet-top {
          top: 10px;
        }
        .leaflet-control-container .leaflet-bottom {
          bottom: 10px;
        }
        .leaflet-control-zoom {
          margin-left: 10px !important;
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
      className="z-0 rounded-md overflow-hidden"
    >
      <TileLayer
        attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
      />
      
      <LayersControl position="topright">
        {/* Base map layers */}
        <LayersControl.BaseLayer checked name="OpenStreetMap">
          <TileLayer 
            url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
            attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
          />
        </LayersControl.BaseLayer>
        
        <LayersControl.BaseLayer name="Satellite">
          <TileLayer
            url="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
            attribution='&copy; Esri &mdash; Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community'
          />
        </LayersControl.BaseLayer>
        
        {/* Building overlays */}
        {buildingOverlays.map((overlay, index) => (
          <LayersControl.Overlay key={index} name={`${overlay.name} Floor Plan`}>
            <LayerGroup>
              {/* Implement custom image overlay for building floor plans */}
              {/* This would require converting floor plans to georeferenced images */}
            </LayerGroup>
          </LayersControl.Overlay>
        ))}
        
        {/* Geofence Zones */}
        <LayersControl.Overlay checked name="Geofence Zones">
          <LayerGroup>
            {geofenceZones.map((zone) => (
              <Circle
                key={zone.id}
                center={zone.center}
                radius={zone.radius}
                pathOptions={{
                  color: zone.color,
                  fillColor: zone.color,
                  fillOpacity: 0.2,
                  weight: 2,
                }}
              >
                <Tooltip permanent direction="center" className="geofence-label">
                  {zone.name}
                </Tooltip>
              </Circle>
            ))}
          </LayerGroup>
        </LayersControl.Overlay>
        
        {/* Historical Paths with Arrows */}
        <LayersControl.Overlay checked name="Historical Paths">
          <LayerGroup>
            {historicalPaths.map((path, pathIndex) => (
              <LayerGroup key={`path-${pathIndex}`}>
                {/* Draw the path line */}
                <Polyline
                  positions={path.path}
                  pathOptions={{
                    color: path.color,
                    weight: 3,
                    opacity: 0.8,
                  }}
                >
                  <Tooltip>
                    {`Path from ${new Date(path.timestamp).toLocaleString()}`}
                  </Tooltip>
                </Polyline>
                
                {/* Add directional arrows along the path */}
                {path.path.length > 1 && path.path.slice(0, -1).map((position, index) => {
                  // Only place arrows every 2nd position to avoid overcrowding
                  if (index % 2 !== 0) return null;
                  
                  const nextPosition = path.path[index + 1];
                  const midPoint: [number, number] = [
                    (position[0] + nextPosition[0]) / 2,
                    (position[1] + nextPosition[1]) / 2
                  ];
                  
                  // Calculate the angle for the arrow
                  const angle = Math.atan2(
                    nextPosition[1] - position[1], 
                    nextPosition[0] - position[0]
                  ) * 180 / Math.PI;
                  
                  return (
                    <Marker
                      key={`arrow-${index}`}
                      position={midPoint}
                      icon={L.divIcon({
                        className: 'path-arrow',
                        html: `<div class="path-arrow-container" style="transform: rotate(${angle}deg)">➔</div>`,
                        iconSize: [20, 20],
                        iconAnchor: [10, 10]
                      })}
                      interactive={false}
                    />
                  );
                })}
              </LayerGroup>
            ))}
          </LayerGroup>
        </LayersControl.Overlay>
      </LayersControl>
      
      {/* Robot markers */}
      {robotsWithLocation.map(robot => (
        <Marker
          key={robot.id}
          position={[
            robot.location!.latitude,
            robot.location!.longitude,
          ]}
          icon={createStatusIcon(robot.status)}
        >
          {showTooltips && (
            <Tooltip direction="top" offset={[0, -8]} opacity={1} permanent={false} className="custom-tooltip">
              <div className="tooltip-title">{robot.name}</div>
              <div className="tooltip-grid">
                <span className="tooltip-label">Status:</span>
                <span className={`tooltip-value tooltip-value-${robot.status}`}>
                  {robot.status.charAt(0).toUpperCase() + robot.status.slice(1)}
                </span>
                <span className="tooltip-label">Battery:</span>
                <span className="tooltip-value">{robot.batteryLevel}%</span>
                <span className="tooltip-label">Temperature:</span>
                <span className="tooltip-value">{robot.temperature}°C</span>
                <span className="tooltip-label">Last Seen:</span>
                <span className="tooltip-value">{new Date(robot.lastHeartbeat).toLocaleTimeString()}</span>
              </div>
            </Tooltip>
          )}
          <Popup>
            <div className="text-sm">
              <strong>{robot.name}</strong><br />
              Battery: {robot.batteryLevel}%<br />
              Temp: {robot.temperature}°C<br />
              Status: <span className={`
                ${robot.status === 'online' ? 'text-green-500' :
                  robot.status === 'warning' ? 'text-yellow-500' : 'text-red-500'}
              `}>{robot.status}</span><br />
              IP: {robot.ipAddress}<br />
              Last Seen: {new Date(robot.lastHeartbeat).toLocaleTimeString()}
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
