
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
import { useEffect } from "react";
import { supabase } from "@/integrations/supabase/client";
import { toast } from "@/components/ui/sonner";

export function Dashboard() {
  const { robots: supabaseRobots, loading } = useRobots();
  const { user } = useAuth();
  
  // Map Supabase robots to application Robot type
  const robots: Robot[] = supabaseRobots.map(mapSupabaseRobotToAppRobot);
  
  // Enable realtime database for tables
  useEffect(() => {
    // This will run just once to set up the database for realtime changes
    const setupRealtime = async () => {
      try {
        // Run SQL commands to ensure tables are setup for realtime
        // Use type assertion to fix TypeScript error with RPC parameters
        await supabase.rpc('enable_realtime_for_table', { table_name: 'robots' } as any);
        await supabase.rpc('enable_realtime_for_table', { table_name: 'telemetry' } as any);
        console.log('Realtime enabled for database tables');
      } catch (error) {
        console.error('Error setting up realtime:', error);
        // This might fail if the RPC doesn't exist yet, but that's ok
      }
    };
    
    setupRealtime();
  }, []);
  
  // Set up realtime subscription for robot updates
  useEffect(() => {
    // Subscribe to realtime changes on the robots table
    const robotsChannel = supabase
      .channel('robots-updates')
      .on('postgres_changes', 
        { event: '*', schema: 'public', table: 'robots' }, 
        (payload) => {
          console.log('Robot update received:', payload);
          if (payload.eventType === 'UPDATE') {
            toast('Robot status updated', {
              description: `${payload.new.name}'s status has been updated`,
              duration: 3000,
            });
          }
        }
      )
      .subscribe();
    
    // Subscribe to realtime changes on the telemetry table
    const telemetryChannel = supabase
      .channel('telemetry-updates')
      .on('postgres_changes', 
        { event: 'INSERT', schema: 'public', table: 'telemetry' }, 
        (payload) => {
          console.log('New telemetry received:', payload);
        }
      )
      .subscribe();

    // Cleanup subscriptions when component unmounts
    return () => {
      supabase.removeChannel(robotsChannel);
      supabase.removeChannel(telemetryChannel);
    };
  }, []);
  
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
