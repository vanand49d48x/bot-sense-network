
import { Json } from "@/integrations/supabase/types";

export interface Robot {
  id: string;
  created_at: string;
  name: string;
  description: string;
  status: 'online' | 'offline' | 'idle' | 'warning' | 'error';
  location: {
    latitude: number;
    longitude: number;
  };
  batteryLevel: number;
  temperature: number;
  telemetryData?: {
    [key: string]: any;
  };
  lastHeartbeat: string;
  apiKey: string;
  robotType: string;
  model?: string;
  ipAddress?: string;
  errorCount?: number;
}

export interface SupabaseRobot {
  id: string;
  created_at: string;
  name: string;
  description: string;
  status: 'online' | 'offline' | 'idle' | 'warning' | 'error';
  location: Json;
  battery_level: number;
  temperature: number;
  telemetry_data: Json;
  last_heartbeat: string;
  api_key: string;
  robot_type: string;
}

export interface Alert {
  id: string;
  created_at: string;
  robot_id: string;
  type: 'battery' | 'temperature' | 'location' | 'custom';
  threshold: number;
  message: string;
  status: 'active' | 'inactive' | 'resolved';
}

export interface UserProfile {
  id?: string;
  first_name?: string;
  last_name?: string;
  avatar_url?: string;
  created_at?: string;
  updated_at?: string;
  api_key?: string;
  custom_robot_types?: string[];
  custom_telemetry_types?: string[];
  custom_alerts?: Json[];
  telemetry_retention_days?: number;
}
