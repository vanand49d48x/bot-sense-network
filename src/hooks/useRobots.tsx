
import { useState, useEffect } from "react";
import { supabase } from "@/integrations/supabase/client";
import { useToast } from "@/hooks/use-toast";
import { Database } from "@/types/supabase";
import { useAuth } from "@/context/AuthContext";
import { SupabaseRobot } from "@/utils/robotMapper";

export function useRobots() {
  const [robots, setRobots] = useState<SupabaseRobot[]>([]);
  const [loading, setLoading] = useState(true);
  const { toast } = useToast();
  const { session } = useAuth();

  useEffect(() => {
    if (!session) return;
    
    const fetchRobots = async () => {
      try {
        setLoading(true);
        const { data, error } = await supabase
          .from('robots')
          .select('*');

        if (error) throw error;
        
        setRobots(data || []);
      } catch (error: any) {
        toast({
          title: "Error fetching robots",
          description: error.message,
          variant: "destructive",
        });
      } finally {
        setLoading(false);
      }
    };

    fetchRobots();
    
    // Set up realtime subscription for robot updates
    const robotsSubscription = supabase
      .channel('robots-changes')
      .on('postgres_changes', 
        { event: '*', schema: 'public', table: 'robots' }, 
        (payload) => {
          console.log("Robots change detected:", payload.eventType);
          
          if (payload.eventType === 'INSERT') {
            setRobots(prev => [...prev, payload.new as SupabaseRobot]);
          } else if (payload.eventType === 'UPDATE') {
            setRobots(prev => 
              prev.map(robot => robot.id === payload.new.id ? payload.new as SupabaseRobot : robot)
            );
          } else if (payload.eventType === 'DELETE') {
            setRobots(prev => 
              prev.filter(robot => robot.id !== payload.old.id)
            );
          }
        })
      .subscribe();

    return () => {
      supabase.removeChannel(robotsSubscription);
    };
  }, [session, toast]);

  const addRobot = async (robot: Omit<Database['public']['Tables']['robots']['Insert'], 'user_id' | 'api_key'>) => {
    try {
      if (!session?.user?.id) throw new Error("User not authenticated");
      
      // Generate a random API key for the robot
      const apiKey = crypto.randomUUID().replace(/-/g, '') + crypto.randomUUID().replace(/-/g, '');
      
      const { data, error } = await supabase
        .from('robots')
        .insert([
          {
            ...robot,
            api_key: apiKey,
            user_id: session.user.id
          }
        ])
        .select();

      if (error) throw error;
      
      toast({
        title: "Robot added",
        description: `${robot.name} has been successfully added.`
      });
      
      return data?.[0];
    } catch (error: any) {
      toast({
        title: "Error adding robot",
        description: error.message,
        variant: "destructive",
      });
      throw error;
    }
  };

  const deleteRobot = async (robotId: string) => {
    try {
      if (!session?.user?.id) throw new Error("User not authenticated");

      const { error } = await supabase
        .from('robots')
        .delete()
        .eq('id', robotId)
        .eq('user_id', session.user.id);

      if (error) throw error;
      
      return true;
    } catch (error: any) {
      console.error("Error deleting robot:", error);
      throw error;
    }
  };

  return { robots, loading, addRobot, deleteRobot };
}
