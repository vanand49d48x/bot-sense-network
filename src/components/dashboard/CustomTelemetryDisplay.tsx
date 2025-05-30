import { Robot } from "@/types/robot";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { AlertCircle } from "lucide-react";

interface CustomTelemetryDisplayProps {
  robot: Robot;
}

export function CustomTelemetryDisplay({ robot }: CustomTelemetryDisplayProps) {
  // Check if robot has telemetry data
  if (!robot.telemetryData || Object.keys(robot.telemetryData).length === 0) {
    return (
      <Card className="mb-4">
        <CardHeader className="pb-2">
          <CardTitle className="text-lg flex items-center">
            <span>Custom Telemetry</span>
            <Badge variant="outline" className="ml-2">
              No Data
            </Badge>
          </CardTitle>
        </CardHeader>
        <CardContent>
          <p className="text-sm text-muted-foreground">
            No custom telemetry data available for this robot. Send custom telemetry using the API.
          </p>
        </CardContent>
      </Card>
    );
  }

  console.log("Custom telemetry data received:", robot.telemetryData);

  // Filter out standard telemetry fields that are already displayed elsewhere
  const standardFields = ["batteryLevel", "temperature", "status", "location", "timestamp"];
  
  // Extract custom telemetry data
  let customTelemetryEntries: [string, any][] = [];
  
  // Check if customTelemetry object exists and extract its entries
  if (robot.telemetryData.customTelemetry && typeof robot.telemetryData.customTelemetry === 'object') {
    customTelemetryEntries = Object.entries(robot.telemetryData.customTelemetry);
    console.log("Extracted custom telemetry entries:", customTelemetryEntries);
  }
  
  // Also include any other non-standard fields at the root level
  const rootLevelCustom = Object.entries(robot.telemetryData).filter(
    ([key]) => !standardFields.includes(key) && key !== 'customTelemetry'
  );
  
  // Combine both sources of custom telemetry
  const allCustomTelemetry = [...customTelemetryEntries, ...rootLevelCustom];

  if (allCustomTelemetry.length === 0) {
    return (
      <Card className="mb-4">
        <CardHeader className="pb-2">
          <CardTitle className="text-lg flex items-center">
            <span>Custom Telemetry</span>
            <Badge variant="outline" className="ml-2">
              No Custom Data
            </Badge>
          </CardTitle>
        </CardHeader>
        <CardContent>
          <p className="text-sm text-muted-foreground">
            Only standard telemetry data is available. Add custom fields to see them here.
          </p>
        </CardContent>
      </Card>
    );
  }

  // Format value based on its type
  const formatValue = (value: any) => {
    if (typeof value === "boolean") {
      return value ? "True" : "False";
    } else if (typeof value === "object") {
      return JSON.stringify(value);
    }
    return String(value);
  };

  // Determine if a value should trigger an alert (used for custom alert handling)
  const isAlertValue = (key: string, value: any) => {
    // This can be enhanced with user-defined thresholds from the profile settings
    if (typeof value === "number") {
      // Add specific handling for ABCD
      if (key === "ABCD" && value > 40) return true;
      
      // Other threshold logic
      if (key.toLowerCase().includes("error") && value > 0) return true;
      if (key.toLowerCase().includes("temperature") && value > 40) return true;
      if (key.toLowerCase().includes("pressure") && value > 100) return true;
    }
    return false;
  };

  return (
    <Card className="mb-4">
      <CardHeader className="pb-2">
        <CardTitle className="text-lg">Custom Telemetry</CardTitle>
      </CardHeader>
      <CardContent>
        <div className="grid grid-cols-2 gap-4">
          {allCustomTelemetry.map(([key, value]) => {
            const isAlert = isAlertValue(key, value);
            return (
              <div 
                key={key} 
                className={`p-2 rounded-md ${isAlert ? 'bg-red-50 dark:bg-red-900/20' : 'bg-muted'}`}
              >
                <div className="flex justify-between items-center">
                  <p className="text-sm font-medium">{key}</p>
                  {isAlert && (
                    <AlertCircle className="h-4 w-4 text-red-500" />
                  )}
                </div>
                <p className={`text-lg ${isAlert ? 'text-red-600 dark:text-red-400' : ''}`}>
                  {formatValue(value)}
                </p>
              </div>
            );
          })}
        </div>
      </CardContent>
    </Card>
  );
}
