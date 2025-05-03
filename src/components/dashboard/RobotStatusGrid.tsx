
import { useState, useEffect } from "react";
import { Robot, UserProfile } from "@/types/robot";
import { RobotStatusCard } from "./RobotStatusCard";
import { Dialog, DialogContent } from "@/components/ui/dialog";
import { RobotDetailView } from "./RobotDetailView";
import { useAuth } from "@/context/AuthContext";
import { supabase } from "@/integrations/supabase/client";
import { useQuery } from "@tanstack/react-query";
import { SupabaseRobot, mapSupabaseRobotToAppRobot } from "@/utils/robotMapper";

interface RobotStatusGridProps {
  robots: Robot[];
}

export function RobotStatusGrid({ robots }: RobotStatusGridProps) {
  const [selectedRobot, setSelectedRobot] = useState<Robot | null>(null);
  const [localRobots, setLocalRobots] = useState<Robot[]>([]);
  const { user } = useAuth();
  
  // Load user profile to get custom telemetry types
  const { data: userProfile } = useQuery({
    queryKey: ['userProfile', user?.id],
    queryFn: async () => {
      if (!user?.id) return null;
      
      const { data, error } = await supabase
        .from('profiles')
        .select('id, api_key, custom_telemetry_types, first_name, last_name, avatar_url, custom_robot_types')
        .eq('id', user.id)
        .single();
        
      if (error) {
        console.error('Error fetching user profile:', error);
        return null;
      }
      
      return data as UserProfile;
    },
    enabled: !!user?.id,
  });

  // Initialize local robots state when props change
  useEffect(() => {
    if (robots.length > 0) {
      setLocalRobots(robots);
    }
  }, [robots]);

  // Set up realtime subscriptions for robot and telemetry updates
  useEffect(() => {
    console.log("Setting up realtime subscriptions in RobotStatusGrid...");
    
    // Create a channel for robot-related updates
    const channel = supabase
      .channel('robot-status-updates')
      // Subscribe to robot changes
      .on('postgres_changes', 
        { event: '*', schema: 'public', table: 'robots' }, 
        (payload) => {
          console.log('Robot update received in RobotStatusGrid:', payload);
          
          // Handle different event types
          if (payload.eventType === 'UPDATE') {
            // Update robot in local state
            setLocalRobots(prevRobots => {
              return prevRobots.map(robot => {
                if (robot.id === payload.new.id) {
                  console.log('Updating robot in RobotStatusGrid:', payload.new);
                  // Cast the payload to SupabaseRobot and use the mapper function
                  const supabaseRobot = payload.new as SupabaseRobot;
                  return mapSupabaseRobotToAppRobot(supabaseRobot);
                }
                return robot;
              });
            });
            
            // If selected robot was updated, update it as well
            if (selectedRobot && selectedRobot.id === payload.new.id) {
              const supabaseRobot = payload.new as SupabaseRobot;
              setSelectedRobot(mapSupabaseRobotToAppRobot(supabaseRobot));
            }
          }
        }
      )
      // Subscribe to telemetry changes to update robot status
      .on('postgres_changes', 
        { event: 'INSERT', schema: 'public', table: 'telemetry' }, 
        (payload) => {
          console.log('New telemetry received in RobotStatusGrid:', payload);
          const robotId = payload.new.robot_id;
          
          // Update the robot with new telemetry data
          setLocalRobots(prevRobots => {
            return prevRobots.map(robot => {
              if (robot.id === robotId) {
                // Create updated robot with new telemetry data
                const updatedRobot = { ...robot };
                
                // Update basic telemetry fields
                if (payload.new.battery_level !== null && payload.new.battery_level !== undefined) {
                  updatedRobot.batteryLevel = payload.new.battery_level;
                }
                
                if (payload.new.temperature !== null && payload.new.temperature !== undefined) {
                  updatedRobot.temperature = payload.new.temperature;
                }
                
                // Update status and last heartbeat
                updatedRobot.status = 'online';
                updatedRobot.lastHeartbeat = new Date().toISOString();
                
                // Update location if provided
                if (payload.new.location) {
                  updatedRobot.location = {
                    latitude: payload.new.location.latitude || 0,
                    longitude: payload.new.location.longitude || 0
                  };
                }
                
                // Update custom telemetry data
                if (payload.new.motor_status) {
                  try {
                    const customData = typeof payload.new.motor_status === 'string' 
                      ? JSON.parse(payload.new.motor_status)
                      : payload.new.motor_status;
                    
                    updatedRobot.telemetryData = {
                      ...updatedRobot.telemetryData,
                      ...customData
                    };
                  } catch (e) {
                    console.error("Error parsing custom telemetry:", e);
                  }
                }
                
                console.log(`Updated robot ${robot.name} with new telemetry in RobotStatusGrid`);
                return updatedRobot;
              }
              return robot;
            });
          });
          
          // Also update selected robot if it's the one receiving telemetry
          if (selectedRobot && selectedRobot.id === robotId) {
            setSelectedRobot(prev => {
              if (!prev) return prev;
              
              const updatedRobot = { ...prev };
              // Apply the same updates as above
              if (payload.new.battery_level !== null && payload.new.battery_level !== undefined) {
                updatedRobot.batteryLevel = payload.new.battery_level;
              }
              
              if (payload.new.temperature !== null && payload.new.temperature !== undefined) {
                updatedRobot.temperature = payload.new.temperature;
              }
              
              updatedRobot.status = 'online';
              updatedRobot.lastHeartbeat = new Date().toISOString();
              
              return updatedRobot;
            });
          }
        }
      )
      .subscribe((status) => {
        console.log(`RobotStatusGrid subscription status: ${status}`);
      });
      
    return () => {
      console.log("Cleaning up realtime subscriptions in RobotStatusGrid");
      supabase.removeChannel(channel);
    };
  }, []); // Empty dependency array to set up subscription once

  if (robots.length === 0) {
    return (
      <div className="text-center p-12 border border-dashed rounded-lg">
        <h3 className="text-lg font-medium">No robots available</h3>
        <p className="text-muted-foreground">Add robots to see their status here.</p>
      </div>
    );
  }

  // Use localRobots for rendering to ensure real-time updates are displayed
  const displayRobots = localRobots.length > 0 ? localRobots : robots;

  return (
    <div>
      <h2 className="text-xl font-semibold mb-4">Robot Status</h2>
      
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
        {displayRobots.map((robot) => (
          <div key={robot.id} onClick={() => setSelectedRobot(robot)} className="cursor-pointer">
            <RobotStatusCard robot={robot} />
          </div>
        ))}
      </div>

      <Dialog open={!!selectedRobot} onOpenChange={(open) => !open && setSelectedRobot(null)}>
        <DialogContent className="sm:max-w-[600px]">
          {selectedRobot && userProfile && <RobotDetailView robot={selectedRobot} userProfile={userProfile} />}
        </DialogContent>
      </Dialog>
    </div>
  );
}
