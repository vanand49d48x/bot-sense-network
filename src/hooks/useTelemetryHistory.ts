
import { useState, useEffect } from "react";
import { supabase } from "@/integrations/supabase/client";
import { format } from "date-fns";
import { toast } from "@/components/ui/sonner";
import { useRobots } from "@/hooks/useRobots";

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
  const { apiKey } = useRobots(); // Use the central API key

  useEffect(() => {
    async function fetchTelemetryHistory() {
      if (!robotId || !apiKey) {
        setLoading(false);
        return;
      }

      try {
        setLoading(true);
        setError(null); // Reset any previous errors

        console.log("Making request to edge function with API key");
        
        // Make a request to our edge function using the central API key
        const response = await fetch(
          `https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/get-telemetry/robots/${robotId}/telemetry?last=${limit}`,
          {
            headers: {
              "api-key": apiKey,
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
  }, [robotId, limit, apiKey]);

  return { telemetry, loading, error };
}
