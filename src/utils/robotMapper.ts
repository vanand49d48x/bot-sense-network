
import { Robot } from "@/types/robot";
import { Database } from "@/types/supabase";

// Type for robots coming from Supabase
export type SupabaseRobot = Database['public']['Tables']['robots']['Row'];

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
    apiKey: robot.api_key, // Add the API key to the robot object
  };
}
