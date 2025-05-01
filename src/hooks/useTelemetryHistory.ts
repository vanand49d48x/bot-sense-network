
import { useState, useEffect } from "react";
import { supabase } from "@/integrations/supabase/client";
import { format } from "date-fns";
import { toast } from "@/components/ui/sonner";

export type TelemetryRecord = {
  id: string;
  robotId: string;
  batteryLevel: number | null;
  temperature: number | null;
  location: { latitude: number; longitude: number } | null;
  timestamp: string;
  status: "OK" | "ERROR" | "WARNING";
  formattedTime?: string;
  formattedDate?: string;
};

export function useTelemetryHistory(robotId: string, limit: number = 1000) {
  const [telemetry, setTelemetry] = useState<TelemetryRecord[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<Error | null>(null);

  useEffect(() => {
    async function fetchTelemetryHistory() {
      if (!robotId) {
        setLoading(false);
        return;
      }

      try {
        setLoading(true);
        setError(null); // Reset any previous errors

        // Get the robot's API key
        const { data: robotData, error: robotError } = await supabase
          .from("robots")
          .select("api_key")
          .eq("id", robotId)
          .single();

        if (robotError || !robotData?.api_key) {
          throw new Error(`Failed to fetch robot API key: ${robotError?.message || "Robot not found"}`);
        }

        console.log("Making request to edge function with API key");
        
        // Make a request to our edge function using the robot's API key
        const response = await fetch(
          `https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/get-telemetry/robots/${robotId}/telemetry?last=${limit}`,
          {
            headers: {
              "apikey": robotData.api_key,
              "Authorization": `Bearer ${robotData.api_key}`, // Add Authorization header for edge function
              "Content-Type": "application/json"
            },
          }
        );

        if (!response.ok) {
          const errorText = await response.text();
          console.error("Failed to fetch telemetry data:", response.statusText, errorText);
          throw new Error(`Failed to fetch telemetry data: ${response.statusText} - ${errorText}`);
        }

        const data = await response.json();
        
        if (data.success && Array.isArray(data.data)) {
          // Process and format the telemetry data
          const formattedTelemetry = data.data.map((record: TelemetryRecord) => ({
            ...record,
            formattedTime: format(new Date(record.timestamp), "HH:mm:ss"),
            formattedDate: format(new Date(record.timestamp), "MMM dd, yyyy"),
          }));
          
          setTelemetry(formattedTelemetry);
        } else {
          console.error("Invalid telemetry data format:", data);
          throw new Error("Invalid telemetry data format");
        }
      } catch (error) {
        console.error("Error fetching telemetry history:", error);
        setError(error instanceof Error ? error : new Error("Failed to fetch telemetry history"));
        toast.error("Failed to load telemetry data");
      } finally {
        setLoading(false);
      }
    }

    fetchTelemetryHistory();
  }, [robotId, limit]);

  return { telemetry, loading, error };
}
