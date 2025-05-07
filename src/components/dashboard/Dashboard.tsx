
import { useEffect, useState } from "react";
import { DashboardHeader } from "./DashboardHeader";
import { StatCards } from "./StatCards";
import { RobotStatusGrid } from "./RobotStatusGrid";
import { MapView } from "./MapView";
import { AddRobotModal } from "./AddRobotModal";
import { useAuth } from "@/context/AuthContext";
import { AlertCircle } from "lucide-react";
import { Alert, AlertDescription, AlertTitle } from "@/components/ui/alert";
import { supabase } from "@/integrations/supabase/client";
import { toast } from "@/components/ui/sonner";
import { Collapsible, CollapsibleContent, CollapsibleTrigger } from "@/components/ui/collapsible";
import { Button } from "@/components/ui/button";
import { ChevronDown, ChevronUp } from "lucide-react";
import { useQuery } from "@tanstack/react-query";
import { useRobotStore } from "@/store/robotStore";
import { RobotFilter } from "./RobotFilter";

export function Dashboard() {
  const { user } = useAuth();
  const { 
    robots, 
    filteredRobots, 
    isLoading, 
    fetchRobots, 
    updateRobotFromTelemetry,
    setFilter 
  } = useRobotStore();
  
  const [isFilterOpen, setIsFilterOpen] = useState(false);
  
  // Load user profile data
  const { data: userProfile } = useQuery({
    queryKey: ['userProfile', user?.id],
    queryFn: async () => {
      if (!user?.id) return null;
      
      const { data, error } = await supabase
        .from('profiles')
        .select('api_key, custom_robot_types, custom_telemetry_types')
        .eq('id', user.id)
        .single();
        
      if (error) {
        console.error('Error fetching user profile:', error);
        return null;
      }
      
      return data;
    },
    enabled: !!user?.id,
  });
  
  // Fetch robots data initially
  useEffect(() => {
    fetchRobots();
  }, [fetchRobots]);
  
  // Handle refresh button click
  const handleRefresh = () => {
    toast('Refreshing robot data...', {
      description: 'Fetching the latest robot status and telemetry',
      duration: 2000,
    });
    fetchRobots();
  };
  
  // Handle filter changes
  const handleFilteredRobotsChange = (filtered) => {
    setFilter('custom', filtered);
  };
  
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
          
          // The robots state is now handled by the robotStore
          // We'll refresh the data to ensure everything is in sync
          fetchRobots();
        }
      )
      // Subscribe to telemetry changes
      .on('postgres_changes', 
        { event: 'INSERT', schema: 'public', table: 'telemetry' }, 
        (payload) => {
          console.log('New telemetry received:', payload);
          const robotId = payload.new.robot_id;
          
          // Create telemetry data object
          const telemetryData = {
            batteryLevel: payload.new.battery_level,
            temperature: payload.new.temperature,
            location: payload.new.location,
          };
          
          // Handle custom telemetry data
          if (payload.new.motor_status) {
            try {
              const customData = typeof payload.new.motor_status === 'string' 
                ? JSON.parse(payload.new.motor_status)
                : payload.new.motor_status;
              
              telemetryData.customTelemetry = customData;
            } catch (e) {
              console.error("Error parsing custom telemetry:", e);
            }
          }
          
          // Update robot state with new telemetry
          updateRobotFromTelemetry(robotId, telemetryData);
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
      
    return () => {
      console.log("Cleaning up realtime subscriptions");
      supabase.removeChannel(channel);
    };
  }, []); // Empty dependency array to run once
  
  if (isLoading) {
    return (
      <div className="flex items-center justify-center min-h-[400px]">
        <div className="text-center">
          <div className="animate-pulse-slow">Loading robots data...</div>
        </div>
      </div>
    );
  }

  // Use filteredRobots from our store
  const displayRobots = filteredRobots;

  return (
    <div>
      <div className="flex justify-between items-center">
        <DashboardHeader onRefresh={handleRefresh} />
        <AddRobotModal />
      </div>
      
      {robots.length > 0 ? (
        <>
          <StatCards robots={displayRobots} />
          
          <Alert className="my-4">
            <AlertCircle className="h-4 w-4" />
            <AlertTitle>Telemetry Integration</AlertTitle>
            <AlertDescription>
              To send telemetry data to your robots, use each robot's API key. View API keys in the robot cards by clicking "API Integration".
              You can now use WebSockets for real-time bidirectional communication!
            </AlertDescription>
          </Alert>
          
          {/* Filter section */}
          <Collapsible
            open={isFilterOpen}
            onOpenChange={setIsFilterOpen}
            className="mt-4 mb-6 border rounded-lg p-4"
          >
            <CollapsibleTrigger asChild>
              <Button variant="ghost" className="flex justify-between w-full">
                <span className="font-medium">Filter Robots</span>
                {isFilterOpen ? <ChevronUp className="h-4 w-4" /> : <ChevronDown className="h-4 w-4" />}
              </Button>
            </CollapsibleTrigger>
            <CollapsibleContent className="mt-4">
              <RobotFilter 
                robots={robots}
                userProfile={userProfile}
                onFilteredRobotsChange={handleFilteredRobotsChange} 
              />
            </CollapsibleContent>
          </Collapsible>
          
          {displayRobots.length !== robots.length && (
            <div className="bg-muted p-2 rounded text-sm mb-4">
              Showing {displayRobots.length} of {robots.length} robots
              {displayRobots.length === 0 && (
                <Button 
                  variant="link" 
                  className="p-0 h-auto text-sm ml-2"
                  onClick={() => useRobotStore.getState().clearFilters()}
                >
                  Clear filters
                </Button>
              )}
            </div>
          )}
          
          <MapView robots={displayRobots} />
          <RobotStatusGrid robots={displayRobots} />
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
