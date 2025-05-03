
import { Robot } from "@/types/robot";
import { Database } from "@/types/supabase";

// Export the SupabaseRobot type so it can be used in other files
export interface SupabaseRobot {
  id: string;
  name: string;
  type: string;
  status: string;
  last_ping?: string;
  battery_level?: number;
  temperature?: number;
  location?: any;
  api_key?: string;
  telemetry_data?: any;
}

/**
 * Maps a Supabase robot to the application Robot type
 */
export function mapSupabaseRobotToAppRobot(robot: SupabaseRobot): Robot {
  return {
    id: robot.id,
    name: robot.name,
    model: robot.type, // Using type as model
    status: robot.status as 'online' | 'offline' | 'warning',
    lastHeartbeat: robot.last_ping || new Date().toISOString(),
    batteryLevel: robot.battery_level || 0,
    temperature: robot.temperature || 0,
    location: robot.location ? {
      latitude: (robot.location as any).latitude || 0,
      longitude: (robot.location as any).longitude || 0
    } : undefined,
    ipAddress: "Unknown", // Default value since Supabase doesn't have this
    errorCount: 0, // Default value since Supabase doesn't track this
    apiKey: robot.api_key, // Map the API key from Supabase
    telemetryData: robot.telemetry_data || {}, // Using type assertion to handle the mismatch
  };
}
