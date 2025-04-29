
import { DashboardHeader } from "./DashboardHeader";
import { StatCards } from "./StatCards";
import { RobotStatusGrid } from "./RobotStatusGrid";
import { MapView } from "./MapView";
import { AddRobotModal } from "./AddRobotModal";
import { useRobots } from "@/hooks/useRobots";
import { useAuth } from "@/context/AuthContext";
import { Robot } from "@/types/robot";
import { mapSupabaseRobotToAppRobot } from "@/utils/robotMapper";
import { AlertCircle } from "lucide-react";
import { Alert, AlertDescription, AlertTitle } from "@/components/ui/alert";
import { useEffect, useState } from "react";
import { supabase } from "@/integrations/supabase/client";
import { toast } from "@/components/ui/sonner";
import { SupabaseRobot } from "@/utils/robotMapper";

export function Dashboard() {
  const { robots: supabaseRobots, loading } = useRobots();
  const { user } = useAuth();
  
  // Map Supabase robots to application Robot type
  const robots: Robot[] = supabaseRobots.map(mapSupabaseRobotToAppRobot);
  
  // Local state to manage robots for real-time updates with a unique key to force re-render on refresh
  const [localRobots, setLocalRobots] = useState<Robot[]>([]);
  const [refreshKey, setRefreshKey] = useState(Date.now());
  
  // Initialize local robots when the data from useRobots changes
  useEffect(() => {
    if (robots.length > 0) {
      console.log("Updating local robots state with", robots.length, "robots");
      setLocalRobots(robots);
      // Update refresh key to force re-render
      setRefreshKey(Date.now());
    }
  }, [robots]);
  
  // Set up realtime subscription for robot and telemetry updates
  useEffect(() => {
    console.log("Setting up realtime subscriptions...");
    
    // Create a single channel for all robot-related updates
    const channel = supabase
      .channel('robot-updates')
      // Subscribe to robot changes
      .on('postgres_changes', 
        { event: '*', schema: 'public', table: 'robots' }, 
        (payload) => {
          console.log('Robot update received in Dashboard:', payload);
          
          // Handle different event types
          if (payload.eventType === 'UPDATE') {
            toast('Robot status updated', {
              description: `${payload.new.name}'s status has been updated`,
              duration: 3000,
            });
            
            // Update the local robots state - ensure we have a valid SupabaseRobot
            setLocalRobots(prevRobots => {
              return prevRobots.map(robot => {
                if (robot.id === payload.new.id) {
                  // Cast the payload to SupabaseRobot and use the mapper function
                  const supabaseRobot = payload.new as SupabaseRobot;
                  return mapSupabaseRobotToAppRobot(supabaseRobot);
                }
                return robot;
              });
            });
            // Update refresh key to force re-render
            setRefreshKey(Date.now());
          } else if (payload.eventType === 'INSERT') {
            toast('New robot added', {
              description: `${payload.new.name} has been added to your fleet`,
              duration: 3000,
            });
            
            // Add the new robot to local state - ensure we have a valid SupabaseRobot
            const supabaseRobot = payload.new as SupabaseRobot;
            setLocalRobots(prevRobots => [
              ...prevRobots, 
              mapSupabaseRobotToAppRobot(supabaseRobot)
            ]);
            // Update refresh key to force re-render
            setRefreshKey(Date.now());
          } else if (payload.eventType === 'DELETE') {
            toast('Robot removed', {
              description: `A robot has been removed from your fleet`,
              duration: 3000,
            });
            
            // Remove the deleted robot from local state
            setLocalRobots(prevRobots => 
              prevRobots.filter(robot => robot.id !== payload.old.id)
            );
            // Update refresh key to force re-render
            setRefreshKey(Date.now());
          }
        }
      )
      // Subscribe to telemetry changes - optimized to patch robot locally
      .on('postgres_changes', 
        { event: 'INSERT', schema: 'public', table: 'telemetry' }, 
        (payload) => {
          console.log('New telemetry received:', payload);
          const robotId = payload.new.robot_id;
          
          // Update the robot immediately with a proper pattern for state updates
          setLocalRobots(prevRobots => {
            // First find the robot to update in the current state
            const robotToUpdate = prevRobots.find(r => r.id === robotId);
            if (!robotToUpdate) return prevRobots; // No matching robot found
            
            // Only patch the specific properties that changed in the telemetry
            const updatedFields: Partial<Robot> = {
              lastHeartbeat: new Date().toISOString(),
              status: 'online', // Set to online since we received telemetry
            };
            
            // Add only the fields that are present in the telemetry payload
            if (payload.new.battery_level !== null && payload.new.battery_level !== undefined) {
              updatedFields.batteryLevel = payload.new.battery_level;
            }
            
            if (payload.new.temperature !== null && payload.new.temperature !== undefined) {
              updatedFields.temperature = payload.new.temperature;
            }
            
            if (payload.new.location) {
              updatedFields.location = {
                latitude: payload.new.location.latitude || 0,
                longitude: payload.new.location.longitude || 0
              };
            }
            
            // Create the updated robots list
            const updatedRobots = prevRobots.map(robot => {
              if (robot.id === robotId) {
                console.log(`Patching robot ${robot.name} with new telemetry data:`, updatedFields);
                return { ...robot, ...updatedFields };
              }
              return robot;
            });
            
            // Show toast with the fresh robot name from the updated list
            const updatedRobot = updatedRobots.find(r => r.id === robotId);
            if (updatedRobot) {
              toast('New telemetry data', {
                description: `${updatedRobot.name} has sent new telemetry data`,
                duration: 2000,
              });
            }

            // Update refresh key to force re-render
            setRefreshKey(Date.now());
            
            return updatedRobots;
          });
        }
      )
      .subscribe((status) => {
        console.log(`Realtime subscription status: ${status}`);
        if (status === 'SUBSCRIBED') {
          console.log('Successfully subscribed to realtime updates');
        } else if (status === 'CHANNEL_ERROR') {
          console.error('Error subscribing to realtime updates');
        }
      });

    // This function enables realtime for tables using a SQL function
    const setupRealtimeTables = async () => {
      try {
        console.log("Enabling realtime for tables via direct SQL approach...");
        
        // Enable realtime for robots table directly
        const { data: robotsData, error: robotsError } = await supabase.from('robots')
          .select('id')
          .limit(1);
          
        if (robotsError) {
          console.error("Error querying robots table:", robotsError);
        } else {
          console.log("Successfully queried robots table for realtime");
        }
        
        // Enable realtime for telemetry table directly
        const { data: telemetryData, error: telemetryError } = await supabase.from('telemetry')
          .select('id')
          .limit(1);
          
        if (telemetryError) {
          console.error("Error querying telemetry table:", telemetryError);
        } else {
          console.log("Successfully queried telemetry table for realtime");
        }
      } catch (error) {
        console.error("Error setting up realtime tables:", error);
      }
    };
    
    // Call the setup function
    setupRealtimeTables();
    
    // Cleanup subscriptions when component unmounts
    return () => {
      console.log("Cleaning up realtime subscriptions");
      supabase.removeChannel(channel);
    };
  }, []); // Remove localRobots from dependency to avoid unnecessary resubscriptions
  
  if (loading) {
    return (
      <div className="flex items-center justify-center min-h-[400px]">
        <div className="text-center">
          <div className="animate-pulse-slow">Loading robots data...</div>
        </div>
      </div>
    );
  }

  return (
    <div>
      <div className="flex justify-between items-center">
        <DashboardHeader />
        <AddRobotModal />
      </div>
      
      {localRobots.length > 0 ? (
        <>
          <StatCards robots={localRobots} key={`stats-${refreshKey}`} />
          
          <Alert className="my-4">
            <AlertCircle className="h-4 w-4" />
            <AlertTitle>Telemetry Integration</AlertTitle>
            <AlertDescription>
              To send telemetry data to your robots, use each robot's API key. View API keys in the robot cards by clicking "API Integration".
            </AlertDescription>
          </Alert>
          
          <MapView robots={localRobots} key={`map-${refreshKey}`} />
          <RobotStatusGrid robots={localRobots} key={`grid-${refreshKey}`} />
        </>
      ) : (
        <div className="mt-8 text-center p-12 border border-dashed rounded-lg">
          <h3 className="text-lg font-medium mb-2">No robots registered yet</h3>
          <p className="text-muted-foreground mb-4">
            Add your first robot to start monitoring its status and telemetry.
          </p>
          <AddRobotModal />
        </div>
      )}
    </div>
  );
}
