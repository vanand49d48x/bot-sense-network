
import { useState, useRef } from "react";
import { Robot, UserProfile } from "@/types/robot";
import { RobotStatusBadge } from "./RobotStatusBadge";
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
} from "@/components/ui/dialog";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Battery, Thermometer, MapPin, Clock, Info, GripHorizontal } from "lucide-react";
import { CustomTelemetryDisplay } from "./CustomTelemetryDisplay";
import { format } from "date-fns";
import { TelemetryHistory } from "./TelemetryHistory";
import { TelemetryChart } from "./TelemetryChart";
import { RobotPathHistory } from "./RobotPathHistory";
import { ScrollArea } from "@/components/ui/scroll-area";
import { ResizablePanelGroup, ResizablePanel, ResizableHandle } from "@/components/ui/resizable";
import { useIsMobile } from "@/hooks/use-mobile";

interface RobotDetailViewProps {
  robot: Robot;
  userProfile: UserProfile | null;
  isOpen: boolean;
  onClose: () => void;
}

export function RobotDetailView({ robot, userProfile, isOpen, onClose }: RobotDetailViewProps) {
  const [activeTab, setActiveTab] = useState("details");
  const isMobile = useIsMobile();
  const dialogRef = useRef<HTMLDivElement>(null);
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const [isDragging, setIsDragging] = useState(false);
  const dragStartPos = useRef({ x: 0, y: 0 });

  const formatDate = (dateString: string) => {
    try {
      return format(new Date(dateString), "MMM d, yyyy h:mm a");
    } catch (e) {
      return "Unknown";
    }
  };

  const retentionDays = userProfile?.telemetry_retention_days || 7;

  const handleMouseDown = (e: React.MouseEvent<HTMLDivElement>) => {
    if (dialogRef.current && e.target === e.currentTarget) {
      setIsDragging(true);
      dragStartPos.current = { 
        x: e.clientX - position.x, 
        y: e.clientY - position.y 
      };
      e.preventDefault();
    }
  };

  const handleMouseMove = (e: React.MouseEvent<HTMLDivElement>) => {
    if (isDragging && dialogRef.current) {
      const newX = e.clientX - dragStartPos.current.x;
      const newY = e.clientY - dragStartPos.current.y;
      setPosition({ x: newX, y: newY });
      e.preventDefault();
    }
  };

  const handleMouseUp = () => {
    setIsDragging(false);
  };

  return (
    <Dialog open={isOpen} onOpenChange={(open) => !open && onClose()}>
      <DialogContent 
        ref={dialogRef}
        className={`min-w-[85vw] sm:min-w-[600px] md:min-w-[700px] p-0 max-w-[90vw] transform-none ${isDragging ? 'cursor-grabbing' : ''}`}
        style={{
          position: 'fixed',
          left: `${position.x}px`, 
          top: `${position.y}px`,
          transform: position.x || position.y ? 'none' : undefined,
        }}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
      >
        <div 
          className="bg-muted h-8 flex items-center px-4 cursor-grab border-b"
          onMouseDown={handleMouseDown}
        >
          <GripHorizontal className="h-4 w-4 mr-2 text-muted-foreground" />
          <span className="text-sm font-medium">{robot.name}</span>
          <div className="ml-auto">
            <RobotStatusBadge status={robot.status} />
          </div>
        </div>
        
        <ResizablePanelGroup direction="vertical" className="w-full">
          <ResizablePanel defaultSize={100} minSize={25}>
            <div className="p-4">
              <Tabs defaultValue="details" value={activeTab} onValueChange={setActiveTab} className="w-full">
                <TabsList className={`grid ${isMobile ? 'grid-cols-2 gap-1' : 'grid-cols-4'} mb-4`}>
                  <TabsTrigger value="details" className="text-xs sm:text-sm">Details</TabsTrigger>
                  <TabsTrigger value="charts" className="text-xs sm:text-sm">Charts</TabsTrigger>
                  <TabsTrigger value="telemetry" className="text-xs sm:text-sm">Custom Telemetry</TabsTrigger>
                  <TabsTrigger value="history" className="text-xs sm:text-sm">History</TabsTrigger>
                </TabsList>
                
                <div className="h-[350px] md:h-[400px]">
                  <TabsContent value="details" className="mt-0 h-full">
                    <ScrollArea className="h-full pr-4">
                      <div className="grid gap-3">
                        <div className="grid grid-cols-1 sm:grid-cols-2 gap-2">
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

                        <div className="grid grid-cols-1 sm:grid-cols-2 gap-3 mt-2">
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
                    </ScrollArea>
                  </TabsContent>
                  
                  <TabsContent value="charts" className="mt-0 h-full">
                    <ScrollArea className="h-full pr-4">
                      <div className="space-y-6">
                        <TelemetryChart robotId={robot.id} retentionDays={retentionDays} />
                        <RobotPathHistory robot={robot} retentionDays={retentionDays} />
                      </div>
                    </ScrollArea>
                  </TabsContent>
                  
                  <TabsContent value="telemetry" className="mt-0 h-full">
                    <ScrollArea className="h-full pr-4">
                      <CustomTelemetryDisplay robot={robot} />
                    </ScrollArea>
                  </TabsContent>
                  
                  <TabsContent value="history" className="mt-0 h-full">
                    <ScrollArea className="h-full pr-4">
                      <TelemetryHistory 
                        robotId={robot.id} 
                        retentionDays={retentionDays}
                      />
                    </ScrollArea>
                  </TabsContent>
                </div>
              </Tabs>
            </div>
          </ResizablePanel>
          <ResizableHandle withHandle />
          <ResizablePanel defaultSize={0} minSize={0} maxSize={30} className="bg-muted">
            <div className="p-4">
              <h3 className="text-sm font-medium mb-2">Robot Information</h3>
              <div className="text-xs text-muted-foreground space-y-1">
                <p>ID: {robot.id}</p>
                <p>Status: {robot.status}</p>
                <p>Last Updated: {formatDate(robot.lastHeartbeat)}</p>
              </div>
            </div>
          </ResizablePanel>
        </ResizablePanelGroup>
      </DialogContent>
    </Dialog>
  );
}
