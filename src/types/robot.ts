
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
  apiKey?: string;
}

export interface UserProfile {
  id: string;
  first_name: string | null;
  last_name: string | null;
  avatar_url: string | null;
  api_key: string | null;
  custom_robot_types: string[] | null;
}
