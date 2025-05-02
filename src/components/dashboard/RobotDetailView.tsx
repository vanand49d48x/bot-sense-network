
import { useState } from "react";
import { Robot, UserProfile } from "@/types/robot";
import { RobotStatusBadge } from "./RobotStatusBadge";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Battery, Thermometer, MapPin, Clock, Info, Code, History } from "lucide-react";
import { CustomTelemetryDisplay } from "./CustomTelemetryDisplay";
import { CustomTelemetryGuide } from "../integration/CustomTelemetryGuide";
import { format } from "date-fns";
import { TelemetryHistory } from "./TelemetryHistory";

interface RobotDetailViewProps {
  robot: Robot;
  userProfile: UserProfile | null;
}

export function RobotDetailView({ robot, userProfile }: RobotDetailViewProps) {
  const [activeTab, setActiveTab] = useState("details");

  const formatDate = (dateString: string) => {
    try {
      return format(new Date(dateString), "MMM d, yyyy h:mm a");
    } catch (e) {
      return "Unknown";
    }
  };

  return (
    <Card className="w-full">
      <CardHeader className="pb-2">
        <div className="flex justify-between items-center">
          <CardTitle className="text-xl">{robot.name}</CardTitle>
          <RobotStatusBadge status={robot.status} />
        </div>
      </CardHeader>
      <CardContent>
        <Tabs defaultValue="details" value={activeTab} onValueChange={setActiveTab} className="w-full">
          <TabsList className="grid grid-cols-4 mb-4">
            <TabsTrigger value="details">Details</TabsTrigger>
            <TabsTrigger value="telemetry">Custom Telemetry</TabsTrigger>
            <TabsTrigger value="history">History</TabsTrigger>
            <TabsTrigger value="integration">API Integration</TabsTrigger>
          </TabsList>
          
          <TabsContent value="details">
            <div className="grid gap-3">
              <div className="grid grid-cols-2 gap-2">
                <div className="flex items-center gap-2">
                  <Info className="h-4 w-4 text-muted-foreground" />
                  <span className="text-sm font-medium">Model:</span>
                  <span className="text-sm">{robot.model}</span>
                </div>
                <div className="flex items-center gap-2">
                  <Clock className="h-4 w-4 text-muted-foreground" />
                  <span className="text-sm font-medium">Last Ping:</span>
                  <span className="text-sm">{formatDate(robot.lastHeartbeat)}</span>
                </div>
              </div>

              <div className="grid grid-cols-2 gap-3 mt-2">
                <div className="bg-muted p-3 rounded-md">
                  <div className="flex items-center">
                    <Battery className={`h-5 w-5 mr-2 ${robot.batteryLevel < 20 ? 'text-red-500' : 'text-green-500'}`} />
                    <span className="font-medium">Battery</span>
                  </div>
                  <p className="text-2xl mt-1">{robot.batteryLevel}%</p>
                </div>
                <div className="bg-muted p-3 rounded-md">
                  <div className="flex items-center">
                    <Thermometer className={`h-5 w-5 mr-2 ${robot.temperature > 35 ? 'text-red-500' : 'text-green-500'}`} />
                    <span className="font-medium">Temperature</span>
                  </div>
                  <p className="text-2xl mt-1">{robot.temperature}°C</p>
                </div>
              </div>

              {robot.location && (
                <div className="bg-muted p-3 rounded-md mt-2">
                  <div className="flex items-center">
                    <MapPin className="h-5 w-5 mr-2 text-blue-500" />
                    <span className="font-medium">Location</span>
                  </div>
                  <p className="text-sm mt-1">
                    Lat: {robot.location.latitude.toFixed(5)}, 
                    Lng: {robot.location.longitude.toFixed(5)}
                  </p>
                </div>
              )}

              <div className="mt-2 text-sm text-muted-foreground">
                <p>IP Address: {robot.ipAddress}</p>
                <p>Error Count: {robot.errorCount}</p>
              </div>
            </div>
          </TabsContent>
          
          <TabsContent value="telemetry">
            <CustomTelemetryDisplay robot={robot} />
          </TabsContent>
          
          <TabsContent value="history">
            <TelemetryHistory 
              robotId={robot.id} 
              retentionDays={userProfile?.telemetryRetentionDays || 7} 
            />
          </TabsContent>
          
          <TabsContent value="integration">
            <CustomTelemetryGuide 
              apiKey={robot.apiKey || userProfile?.api_key || null} 
              robotId={robot.id} 
              customTelemetryTypes={userProfile?.custom_telemetry_types} 
            />
          </TabsContent>
        </Tabs>
      </CardContent>
    </Card>
  );
}
