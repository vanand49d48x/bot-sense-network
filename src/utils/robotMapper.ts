
import { Robot, SupabaseRobot } from "@/types/robot";

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
    telemetryData: (robot as any).telemetry_data || {}, // Using type assertion to handle the mismatch
  };
}

// Export the SupabaseRobot type for reuse in other files
export { SupabaseRobot } from "@/types/robot";
