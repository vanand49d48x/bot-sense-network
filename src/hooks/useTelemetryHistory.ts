
import { useState, useEffect } from "react";
import { supabase } from "@/integrations/supabase/client";
import { format } from "date-fns";

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

        // Make a request to our edge function that gets telemetry data
        const response = await fetch(
          `https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/get-telemetry/robots/${robotId}/telemetry?last=${limit}`,
          {
            headers: {
              "api-key": (await supabase.auth.getSession()).data.session?.access_token || "",
            },
          }
        );

        if (!response.ok) {
          throw new Error(`Failed to fetch telemetry data: ${response.statusText}`);
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
          throw new Error("Invalid telemetry data format");
        }
      } catch (error) {
        console.error("Error fetching telemetry history:", error);
        setError(error instanceof Error ? error : new Error("Failed to fetch telemetry history"));
      } finally {
        setLoading(false);
      }
    }

    fetchTelemetryHistory();
  }, [robotId, limit]);

  return { telemetry, loading, error };
}
