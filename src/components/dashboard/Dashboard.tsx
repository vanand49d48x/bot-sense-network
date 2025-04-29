
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

export function Dashboard() {
  const { robots: supabaseRobots, loading } = useRobots();
  const { user } = useAuth();
  const [realtimeConfigured, setRealtimeConfigured] = useState(false);
  
  // Map Supabase robots to application Robot type
  const robots: Robot[] = supabaseRobots.map(mapSupabaseRobotToAppRobot);
  
  // Enable realtime database for tables
  useEffect(() => {
    // This will run just once to set up the database for realtime changes
    const setupRealtime = async () => {
      try {
        // Run SQL commands to ensure tables are setup for realtime
        console.log("Setting up realtime functionality for tables...");
        
        // Fix: Using proper type assertion with Record<string, any>
        const robotsResult = await supabase.rpc(
          'enable_realtime_for_table', 
          { table_name: 'robots' } as Record<string, any>
        );
        
        const telemetryResult = await supabase.rpc(
          'enable_realtime_for_table', 
          { table_name: 'telemetry' } as Record<string, any>
        );
        
        if (robotsResult.error) {
          console.error('Error enabling realtime for robots table:', robotsResult.error);
        } else {
          console.log('Realtime enabled for robots table');
        }
        
        if (telemetryResult.error) {
          console.error('Error enabling realtime for telemetry table:', telemetryResult.error);
        } else {
          console.log('Realtime enabled for telemetry table');
        }
        
        setRealtimeConfigured(true);
      } catch (error) {
        console.error('Error setting up realtime:', error);
        // This might fail if the RPC doesn't exist yet, but that's ok
      }
    };
    
    setupRealtime();
  }, []);
  
  // Set up realtime subscription for robot and telemetry updates
  useEffect(() => {
    if (!realtimeConfigured) return;
    
    console.log("Setting up realtime subscriptions...");
    
    // Create a single channel for all robot-related updates
    const channel = supabase
      .channel('robot-updates')
      // Subscribe to robot changes
      .on('postgres_changes', 
        { event: '*', schema: 'public', table: 'robots' }, 
        (payload) => {
          console.log('Robot update received:', payload);
          
          // Handle different event types
          if (payload.eventType === 'UPDATE') {
            toast('Robot status updated', {
              description: `${payload.new.name}'s status has been updated`,
              duration: 3000,
            });
          } else if (payload.eventType === 'INSERT') {
            toast('New robot added', {
              description: `${payload.new.name} has been added to your fleet`,
              duration: 3000,
            });
          } else if (payload.eventType === 'DELETE') {
            toast('Robot removed', {
              description: `A robot has been removed from your fleet`,
              duration: 3000,
            });
          }
        }
      )
      // Subscribe to telemetry changes
      .on('postgres_changes', 
        { event: 'INSERT', schema: 'public', table: 'telemetry' }, 
        (payload) => {
          console.log('New telemetry received:', payload);
          const robotId = payload.new.robot_id;
          
          // Find the robot name
          const robot = supabaseRobots.find(r => r.id === robotId);
          if (robot) {
            toast('New telemetry data', {
              description: `${robot.name} has sent new telemetry data`,
              duration: 2000,
            });
          }
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
    
    // Cleanup subscriptions when component unmounts
    return () => {
      console.log("Cleaning up realtime subscriptions");
      supabase.removeChannel(channel);
    };
  }, [realtimeConfigured, supabaseRobots]);
  
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
      
      {robots.length > 0 ? (
        <>
          <StatCards robots={robots} />
          
          <Alert className="my-4">
            <AlertCircle className="h-4 w-4" />
            <AlertTitle>Telemetry Integration</AlertTitle>
            <AlertDescription>
              To send telemetry data to your robots, use each robot's API key. View API keys in the robot cards by clicking "API Integration".
            </AlertDescription>
          </Alert>
          
          <MapView robots={robots} />
          <RobotStatusGrid robots={robots} />
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
