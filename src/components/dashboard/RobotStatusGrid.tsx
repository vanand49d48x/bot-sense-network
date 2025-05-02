
import { useState } from "react";
import { Robot, UserProfile } from "@/types/robot";
import { RobotStatusCard } from "./RobotStatusCard";
import { Dialog, DialogContent } from "@/components/ui/dialog";
import { RobotDetailView } from "./RobotDetailView";
import { useAuth } from "@/context/AuthContext";
import { supabase } from "@/integrations/supabase/client";
import { useQuery } from "@tanstack/react-query";

interface RobotStatusGridProps {
  robots: Robot[];
}

export function RobotStatusGrid({ robots }: RobotStatusGridProps) {
  const [selectedRobot, setSelectedRobot] = useState<Robot | null>(null);
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

  if (robots.length === 0) {
    return (
      <div className="text-center p-12 border border-dashed rounded-lg">
        <h3 className="text-lg font-medium">No robots available</h3>
        <p className="text-muted-foreground">Add robots to see their status here.</p>
      </div>
    );
  }

  return (
    <div>
      <h2 className="text-xl font-semibold mb-4">Robot Status</h2>
      
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
        {robots.map((robot) => (
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
