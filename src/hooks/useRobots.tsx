
import { useState, useEffect } from "react";
import { supabase } from "@/integrations/supabase/client";
import { useToast } from "@/hooks/use-toast";
import { Database } from "@/types/supabase";
import { useAuth } from "@/context/AuthContext";
import { SupabaseRobot } from "@/utils/robotMapper";

export function useRobots() {
  const [robots, setRobots] = useState<SupabaseRobot[]>([]);
  const [loading, setLoading] = useState(true);
  const [apiKey, setApiKey] = useState<string | null>(null);
  const [isAdmin, setIsAdmin] = useState(false);
  const { toast } = useToast();
  const { session } = useAuth();

  // Check admin status
  useEffect(() => {
    const checkAdmin = async () => {
      if (!session?.user) {
        setIsAdmin(false);
        return;
      }

      try {
        const { data, error } = await supabase
          .rpc('check_if_admin', { user_id: session.user.id });
        
        if (error) {
          console.error("Error checking admin status:", error);
          setIsAdmin(false);
        } else {
          setIsAdmin(!!data);
        }
      } catch (error) {
        console.error("Error checking admin status:", error);
        setIsAdmin(false);
      }
    };

    checkAdmin();
  }, [session]);

  // Define fetchRobots here so it can be exported and used in Dashboard.tsx
  const fetchRobots = async () => {
    if (!session) {
      setRobots([]);
      setLoading(false);
      return;
    }
    
    try {
      setLoading(true);
      console.log("Fetching robots data...");
      
      let robotQuery;
      
      // Admin can see all robots, regular users only see their own
      if (isAdmin) {
        robotQuery = supabase.from('robots').select('*');
      } else {
        robotQuery = supabase.from('robots')
          .select('*')
          .eq('user_id', session.user.id);
      }
      
      const { data, error } = await robotQuery;

      if (error) {
        console.error("Error fetching robots:", error);
        toast({
          title: "Error fetching robots",
          description: error.message,
          variant: "destructive",
        });
        setRobots([]);
      } else {
        console.log("Robots data fetched:", data?.length || 0, "robots");
        setRobots(data || []);
      }
      
      // Fetch API key from profiles
      try {
        const { data: profileData, error: profileError } = await supabase
          .from('profiles')
          .select('api_key')
          .eq('id', session.user.id)
          .single();
          
        if (profileError) {
          console.error("Error fetching API key:", profileError);
        } else {
          setApiKey(profileData?.api_key || null);
          
          // If no API key exists, generate one
          if (!profileData?.api_key) {
            try {
              const newApiKey = crypto.randomUUID().replace(/-/g, '') + crypto.randomUUID().replace(/-/g, '');
              
              const { error: updateError } = await supabase
                .from('profiles')
                .update({ api_key: newApiKey })
                .eq('id', session.user.id);
                
              if (!updateError) {
                setApiKey(newApiKey);
              } else {
                console.error("Error setting API key:", updateError);
              }
            } catch (keyError) {
              console.error("Error generating API key:", keyError);
            }
          }
        }
      } catch (profileFetchError) {
        console.error("Error in profile fetch:", profileFetchError);
      }
    } catch (error: any) {
      console.error("Error fetching robots:", error.message);
      setRobots([]); // Set empty array to prevent infinite loading
    } finally {
      setLoading(false);
    }
  };

  // Fetch robots data and set up real-time subscription
  useEffect(() => {
    if (!session) {
      setRobots([]);
      setLoading(false);
      return;
    }
    
    fetchRobots();
    
    // Set up realtime subscription for robot updates
    const robotsChannel = supabase
      .channel('robots-changes')
      .on('postgres_changes', 
        { event: '*', schema: 'public', table: 'robots' }, 
        (payload) => {
          console.log("Robots change detected in useRobots hook:", payload.eventType, payload);
          
          if (payload.eventType === 'INSERT') {
            // Only add if admin or if the robot belongs to the current user
            if (isAdmin || payload.new.user_id === session.user.id) {
              // Add new robot to the state
              setRobots(prev => [...prev, payload.new as SupabaseRobot]);
              console.log(`Robot inserted: ${payload.new.name}`);
            }
          } else if (payload.eventType === 'UPDATE') {
            // Update existing robot in the state
            setRobots(prev => 
              prev.map(robot => {
                if (robot.id === payload.new.id) {
                  console.log(`Robot updated: ${payload.new.name}`);
                  return payload.new as SupabaseRobot;
                }
                return robot;
              })
            );
          } else if (payload.eventType === 'DELETE') {
            // Remove deleted robot from the state
            setRobots(prev => {
              console.log(`Robot deleted: ID ${payload.old.id}`);
              return prev.filter(robot => robot.id !== payload.old.id);
            });
          }
        })
      .subscribe((status) => {
        console.log(`useRobots hook subscription status: ${status}`);
        if (status === 'SUBSCRIBED') {
          console.log('Successfully subscribed to robots table changes');
        } else if (status === 'CHANNEL_ERROR') {
          console.error('Error subscribing to robots table changes');
        }
      });

    return () => {
      console.log("Cleaning up realtime subscription in useRobots hook");
      supabase.removeChannel(robotsChannel);
    };
  }, [session, toast, isAdmin]);

  const addRobot = async (robot: Omit<Database['public']['Tables']['robots']['Insert'], 'user_id'>) => {
    try {
      if (!session?.user?.id) throw new Error("User not authenticated");
      
      // Create a new robot with required api_key
      const { data, error } = await supabase
        .from('robots')
        .insert([
          {
            ...robot,
            user_id: session.user.id,
            api_key: crypto.randomUUID().replace(/-/g, '')
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

  // Export the fetchRobots function along with other values
  return { robots, loading, apiKey, isAdmin, addRobot, deleteRobot, fetchRobots };
}
