
export interface Robot {
  id: string;
  name: string;
  model: string;
  status: 'online' | 'offline' | 'warning';
  lastHeartbeat: string;
  batteryLevel: number;
  temperature: number;
  location?: {
    lat: number;
    lng: number;
  };
  ipAddress: string;
  errorCount: number;
}
