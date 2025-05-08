import { useEffect, useState } from "react";
import { DashboardHeader } from "./DashboardHeader";
import { StatCards } from "./StatCards";
import { RobotStatusGrid } from "./RobotStatusGrid";
import { MapView } from "./MapView";
import { AddRobotModal } from "./AddRobotModal";
import { useRobots } from "@/hooks/useRobots";
import { useAuth } from "@/context/AuthContext";
import { Robot, UserProfile } from "@/types/robot";
import { mapSupabaseRobotToAppRobot } from "@/utils/robotMapper";
import { AlertCircle } from "lucide-react";
import { Alert, AlertDescription, AlertTitle } from "@/components/ui/alert";
import { supabase } from "@/integrations/supabase/client";
import { toast } from "@/components/ui/sonner";
import { SupabaseRobot } from "@/utils/robotMapper";
import { RobotFilter } from "./RobotFilter";
import { Collapsible, CollapsibleContent, CollapsibleTrigger } from "@/components/ui/collapsible";
import { Button } from "@/components/ui/button";
import { ChevronDown, ChevronUp } from "lucide-react";
import { useQuery } from "@tanstack/react-query";
import { RobotLimitAlert } from "./RobotLimitAlert";
import { useSubscriptionLimits } from "@/utils/planRestrictions";
import { PlanFeatureAlert } from "./PlanFeatureAlert";

export function Dashboard() {
  const { robots: supabaseRobots, loading, fetchRobots } = useRobots();
  const { user } = useAuth();
  
  // Local state to manage robots for real-time updates
  const [localRobots, setLocalRobots] = useState<Robot[]>([]);
  const [filteredRobots, setFilteredRobots] = useState<Robot[]>([]);
  const [isFilterOpen, setIsFilterOpen] = useState(false);
  const [isInitialized, setIsInitialized] = useState(false);
  
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
  
  // Get subscription limits
  const { limits, planName, isActive, isTrialExpired } = useSubscriptionLimits();
  
  // Initialize local robots when the data from useRobots changes
  useEffect(() => {
    if (supabaseRobots.length > 0 && !isInitialized) {
      console.log("Initializing local robots state with", supabaseRobots.length, "robots");
      const mapped = supabaseRobots.map(mapSupabaseRobotToAppRobot);
      setLocalRobots(mapped);
      setFilteredRobots(mapped);
      setIsInitialized(true);
    }
  }, [supabaseRobots, isInitialized]);
  
  // Handle refresh button click
  const handleRefresh = () => {
    toast('Refreshing robot data...', {
      description: 'Fetching the latest robot status and telemetry',
      duration: 2000,
    });
    fetchRobots();
  };
  
  // Handle filter changes
  const handleFilteredRobotsChange = (filtered: Robot[]) => {
    setFilteredRobots(filtered);
  };
  
  // Set up realtime subscription for robot and telemetry updates
  useEffect(() => {
    if (!isInitialized) return; // Don't set up subscriptions until initial data is loaded
    
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
            setLocalRobots(prevRobots => {
              return prevRobots.map(robot => {
                if (robot.id === payload.new.id) {
                  console.log('Updating robot in local state:', payload.new);
                  const supabaseRobot = payload.new as SupabaseRobot;
                  return mapSupabaseRobotToAppRobot(supabaseRobot);
                }
                return robot;
              });
            });
          } else if (payload.eventType === 'INSERT') {
            const supabaseRobot = payload.new as SupabaseRobot;
            setLocalRobots(prevRobots => [
              ...prevRobots, 
              mapSupabaseRobotToAppRobot(supabaseRobot)
            ]);
          } else if (payload.eventType === 'DELETE') {
            setLocalRobots(prevRobots => 
              prevRobots.filter(robot => robot.id !== payload.old.id)
            );
          }
        }
      )
      // Subscribe to telemetry changes
      .on('postgres_changes', 
        { event: 'INSERT', schema: 'public', table: 'telemetry' }, 
        (payload) => {
          console.log('New telemetry received:', payload);
          const robotId = payload.new.robot_id;
          
          setLocalRobots(prevRobots => {
            const robotToUpdate = prevRobots.find(r => r.id === robotId);
            if (!robotToUpdate) return prevRobots;
            
            const updatedFields: Partial<Robot> = {
              lastHeartbeat: new Date().toISOString(),
              status: 'online',
            };
            
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
            
            if (robotToUpdate.telemetryData && payload.new.motor_status) {
              try {
                const customData = typeof payload.new.motor_status === 'string' 
                  ? JSON.parse(payload.new.motor_status)
                  : payload.new.motor_status;
                
                updatedFields.telemetryData = {
                  ...robotToUpdate.telemetryData,
                  ...customData
                };
              } catch (e) {
                console.error("Error parsing custom telemetry:", e);
              }
            }
            
            return prevRobots.map(robot => {
              if (robot.id === robotId) {
                console.log(`Patching robot ${robot.name} with new telemetry data:`, updatedFields);
                return { ...robot, ...updatedFields };
              }
              return robot;
            });
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
      
    return () => {
      console.log("Cleaning up realtime subscriptions");
      supabase.removeChannel(channel);
    };
  }, [isInitialized]); // Only set up subscriptions after initialization
  
  if (loading || !isInitialized) {
    return (
      <div className="flex items-center justify-center min-h-[400px]">
        <div className="text-center">
          <div className="animate-pulse-slow">Loading robots data...</div>
        </div>
      </div>
    );
  }

  // Important: render using localRobots which contains the real-time updates
  const displayRobots = filteredRobots.length > 0 ? filteredRobots : localRobots;
  
  // Check if user has exceeded robot limit
  const isOverRobotLimit = localRobots.length >= limits.robotLimit;
  const isAtRobotLimit = localRobots.length >= limits.robotLimit;
  
  // If over limit, only display the allowed number of robots
  const allowedRobots = isOverRobotLimit 
    ? localRobots.slice(0, limits.robotLimit)
    : localRobots;

  return (
    <div>
      <div className="flex justify-between items-center">
        <DashboardHeader onRefresh={handleRefresh} />
        <AddRobotModal disabled={isAtRobotLimit} />
      </div>
      
      {/* Plan limit warnings */}
      {localRobots.length > 0 && (
        <RobotLimitAlert currentRobotCount={localRobots.length} />
      )}
      
      {isTrialExpired && (
        <PlanFeatureAlert
          title="Free Tier Expired"
          description="Your trial period has expired. Upgrade to continue accessing all features."
          icon={AlertCircle}
        />
      )}
      
      {localRobots.length > 0 ? (
        <>
          <StatCards robots={displayRobots} />
          <div className="mt-4">
            <RobotStatusGrid robots={allowedRobots} />
          </div>
          <div className="mt-4">
            <MapView robots={allowedRobots} />
          </div>
        </>
      ) : (
        <div className="text-center py-8">
          <p className="text-muted-foreground">No robots found. Add your first robot to get started.</p>
        </div>
      )}
    </div>
  );
}
