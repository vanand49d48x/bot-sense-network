
import { useState, useEffect } from "react";
import { supabase } from "@/integrations/supabase/client";
import { Robot } from "@/types/robot";
import { mapSupabaseRobotToAppRobot } from "@/utils/robotMapper";
import { toast } from "@/components/ui/sonner";

export function useRobotDetails(robotId: string) {
  const [robot, setRobot] = useState<Robot | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<Error | null>(null);

  useEffect(() => {
    async function fetchRobotDetails() {
      if (!robotId) {
        setLoading(false);
        return;
      }

      try {
        setLoading(true);
        const { data, error } = await supabase
          .from("robots")
          .select("*")
          .eq("id", robotId)
          .single();

        if (error) {
          throw error;
        }

        if (data) {
          setRobot(mapSupabaseRobotToAppRobot(data));
        }
      } catch (error) {
        console.error("Error fetching robot details:", error);
        setError(error instanceof Error ? error : new Error("Failed to fetch robot details"));
        toast.error("Failed to load robot details");
      } finally {
        setLoading(false);
      }
    }

    fetchRobotDetails();
  }, [robotId]);

  return { robot, loading, error };
}
