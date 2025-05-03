
import { Robot } from "@/types/robot";

// Define the SupabaseRobot type directly here since it's not exported from types/robot.ts
export type SupabaseRobot = {
  id: string;
  name: string;
  robot_type: string;
  status: string;
  last_heartbeat?: string;
  battery_level?: number;
  temperature?: number;
  location?: {
    latitude: number;
    longitude: number;
  };
  api_key?: string;
  created_at: string;
  description?: string;
  telemetry_data?: Record<string, any>;
};

/**
 * Maps a Supabase robot to the application Robot type
 */
export function mapSupabaseRobotToAppRobot(robot: SupabaseRobot): Robot {
  return {
    id: robot.id,
    name: robot.name,
    model: robot.robot_type, // Using robot_type as model
    status: robot.status as 'online' | 'offline' | 'warning' | 'error' | 'idle',
    lastHeartbeat: robot.last_heartbeat || new Date().toISOString(),
    batteryLevel: robot.battery_level || 0,
    temperature: robot.temperature || 0,
    location: robot.location ? {
      latitude: (robot.location as any).latitude || 0,
      longitude: (robot.location as any).longitude || 0
    } : { latitude: 0, longitude: 0 },
    ipAddress: "Unknown", // Default value since Supabase doesn't have this
    errorCount: 0, // Default value since Supabase doesn't track this
    apiKey: robot.api_key, // Map the API key from Supabase
    robotType: robot.robot_type,
    created_at: robot.created_at,
    description: robot.description,
    telemetryData: robot.telemetry_data || {}, // Using the correct field
  };
}
