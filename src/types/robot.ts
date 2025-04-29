
export interface Robot {
  id: string;
  name: string;
  model: string;
  status: 'online' | 'offline' | 'warning';
  lastHeartbeat: string;
  batteryLevel: number;
  temperature: number;
  location?: {
    latitude: number;
    longitude: number;
  };
  ipAddress: string;
  errorCount: number;
}
