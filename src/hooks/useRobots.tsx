
import { useState, useEffect } from "react";
import { supabase } from "@/integrations/supabase/client";
import { useToast } from "@/hooks/use-toast";
import { Database } from "@/types/supabase";
import { useAuth } from "@/context/AuthContext";

export type Robot = Database['public']['Tables']['robots']['Row'];

export function useRobots() {
  const [robots, setRobots] = useState<Robot[]>([]);
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
          if (payload.eventType === 'INSERT') {
            setRobots(prev => [...prev, payload.new as Robot]);
          } else if (payload.eventType === 'UPDATE') {
            setRobots(prev => 
              prev.map(robot => robot.id === payload.new.id ? payload.new as Robot : robot)
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

  const addRobot = async (robot: Omit<Database['public']['Tables']['robots']['Insert'], 'user_id'>) => {
    try {
      if (!session?.user?.id) throw new Error("User not authenticated");
      
      const { data, error } = await supabase
        .from('robots')
        .insert([
          {
            ...robot,
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

  return { robots, loading, addRobot };
}
