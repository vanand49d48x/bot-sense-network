import { useState } from "react";
import { useTelemetryHistory, TelemetryRecord } from "@/hooks/useTelemetryHistory";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { ChartContainer, ChartTooltip, ChartTooltipContent, ChartLegend } from "@/components/ui/chart";
import { Separator } from "@/components/ui/separator";
import { Loader2, ChartLine, Battery, Thermometer, MapPin, Clock, List } from "lucide-react";
import { format } from "date-fns";
import { LineChart, Line, XAxis, YAxis, CartesianGrid, ResponsiveContainer, AreaChart, Area } from "recharts";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { TelemetryTable } from "./TelemetryTable";

interface RobotTelemetryHistoryProps {
  robotId: string;
}

export function RobotTelemetryHistory({ robotId }: RobotTelemetryHistoryProps) {
  const [dataLimit, setDataLimit] = useState(100);
  const { telemetry, loading, error } = useTelemetryHistory(robotId, dataLimit);
  const [activeTab, setActiveTab] = useState("charts");

  // Calculate insights
  const insights = calculateInsights(telemetry);

  if (loading) {
    return (
      <div className="flex justify-center items-center h-64">
        <div className="flex flex-col items-center gap-2">
          <Loader2 className="h-8 w-8 animate-spin text-primary" />
          <p className="text-muted-foreground">Loading telemetry history...</p>
        </div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="flex justify-center items-center h-64">
        <div className="flex flex-col items-center gap-2 text-center">
          <p className="text-destructive">Failed to load telemetry history</p>
          <p className="text-sm text-muted-foreground">Please try again later</p>
        </div>
      </div>
    );
  }

  return (
    <div className="space-y-6">
      <div className="flex justify-between items-center">
        <h2 className="text-2xl font-semibold">Telemetry History</h2>
        <div className="flex items-center gap-2">
          <span className="text-sm text-muted-foreground">Data points:</span>
          <Select 
            value={dataLimit.toString()} 
            onValueChange={(value) => setDataLimit(parseInt(value))}
          >
            <SelectTrigger className="w-[130px]">
              <SelectValue placeholder="100 records" />
            </SelectTrigger>
            <SelectContent className="z-[10000]">
              <SelectItem value="50">Last 50</SelectItem>
              <SelectItem value="100">Last 100</SelectItem>
              <SelectItem value="500">Last 500</SelectItem>
              <SelectItem value="1000">Last 1000</SelectItem>
            </SelectContent>
          </Select>
        </div>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        <InsightCard 
          title="Battery"
          value={insights.averageBattery ? `${insights.averageBattery}%` : "N/A"}
          change={insights.batteryTrend}
          icon={<Battery className="h-4 w-4" />}
        />
        <InsightCard 
          title="Temperature"
          value={insights.averageTemperature ? `${insights.averageTemperature}°C` : "N/A"}
          change={insights.temperatureTrend}
          icon={<Thermometer className="h-4 w-4" />}
        />
        <InsightCard 
          title="Uptime"
          value={insights.uptime}
          change={null}
          icon={<Clock className="h-4 w-4" />}
        />
      </div>

      <Tabs value={activeTab} onValueChange={setActiveTab} className="w-full">
        <TabsList className="grid w-full md:w-[400px] grid-cols-2">
          <TabsTrigger value="charts">
            <ChartLine className="mr-2 h-4 w-4" />
            Charts
          </TabsTrigger>
          <TabsTrigger value="table">
            <List className="mr-2 h-4 w-4" />
            Data Table
          </TabsTrigger>
        </TabsList>

        <TabsContent value="charts" className="mt-6">
          <div className="grid grid-cols-1 gap-6">
            <Card>
              <CardHeader>
                <CardTitle>Battery Level Over Time</CardTitle>
                <CardDescription>Historical battery level of the robot</CardDescription>
              </CardHeader>
              <CardContent>
                <div className="h-[300px]">
                  <ChartContainer 
                    config={{
                      battery: { label: "Battery Level", theme: { light: "#3b82f6", dark: "#60a5fa" } },
                    }}
                  >
                    <AreaChart data={telemetry.slice().reverse()}>
                      <defs>
                        <linearGradient id="batteryGradient" x1="0" y1="0" x2="0" y2="1">
                          <stop offset="5%" stopColor="#3b82f6" stopOpacity={0.8}/>
                          <stop offset="95%" stopColor="#3b82f6" stopOpacity={0.1}/>
                        </linearGradient>
                      </defs>
                      <XAxis 
                        dataKey="formattedTime" 
                        tick={{ fontSize: 12 }}
                        interval="preserveStartEnd"
                        minTickGap={50}
                      />
                      <YAxis 
                        domain={[0, 100]} 
                        tick={{ fontSize: 12 }} 
                        tickFormatter={(value) => `${value}%`}
                      />
                      <CartesianGrid strokeDasharray="3 3" stroke="var(--border)" opacity={0.3} />
                      <ChartTooltip
                        content={<ChartTooltipContent />}
                      />
                      <Area 
                        type="monotone" 
                        dataKey="batteryLevel" 
                        name="battery" 
                        stroke="#3b82f6" 
                        fillOpacity={1} 
                        fill="url(#batteryGradient)" 
                      />
                    </AreaChart>
                  </ChartContainer>
                </div>
              </CardContent>
            </Card>

            <Card>
              <CardHeader>
                <CardTitle>Temperature History</CardTitle>
                <CardDescription>Temperature readings over time</CardDescription>
              </CardHeader>
              <CardContent>
                <div className="h-[300px]">
                  <ChartContainer 
                    config={{
                      temperature: { label: "Temperature (°C)", theme: { light: "#ef4444", dark: "#f87171" } },
                    }}
                  >
                    <LineChart data={telemetry.slice().reverse()}>
                      <XAxis 
                        dataKey="formattedTime" 
                        tick={{ fontSize: 12 }}
                        interval="preserveStartEnd"
                        minTickGap={50}
                      />
                      <YAxis 
                        tick={{ fontSize: 12 }} 
                        tickFormatter={(value) => `${value}°C`}
                      />
                      <CartesianGrid strokeDasharray="3 3" stroke="var(--border)" opacity={0.3} />
                      <ChartTooltip
                        content={<ChartTooltipContent />}
                      />
                      <Line 
                        type="monotone" 
                        dataKey="temperature" 
                        name="temperature" 
                        stroke="#ef4444" 
                        strokeWidth={2}
                        dot={{ r: 0 }}
                        activeDot={{ r: 6 }}
                      />
                    </LineChart>
                  </ChartContainer>
                </div>
              </CardContent>
            </Card>

            <Card>
              <CardHeader>
                <CardTitle>Status Distribution</CardTitle>
                <CardDescription>Breakdown of robot status events</CardDescription>
              </CardHeader>
              <CardContent>
                <div className="flex flex-col items-center justify-center gap-4">
                  <div className="flex gap-4">
                    <StatusBadge count={insights.statusCounts.OK} status="OK" />
                    <StatusBadge count={insights.statusCounts.WARNING} status="WARNING" />
                    <StatusBadge count={insights.statusCounts.ERROR} status="ERROR" />
                  </div>
                  <p className="text-sm text-muted-foreground">
                    {insights.statusSummary}
                  </p>
                </div>
              </CardContent>
            </Card>
          </div>
        </TabsContent>

        <TabsContent value="table">
          <TelemetryTable telemetry={telemetry} />
        </TabsContent>
      </Tabs>
    </div>
  );
}

interface InsightCardProps {
  title: string;
  value: string | number;
  change: number | null;
  icon: React.ReactNode;
}

function InsightCard({ title, value, change, icon }: InsightCardProps) {
  return (
    <Card>
      <CardHeader className="pb-2">
        <div className="flex items-center justify-between">
          <CardTitle className="text-sm font-medium">{title}</CardTitle>
          {icon}
        </div>
      </CardHeader>
      <CardContent>
        <div className="text-2xl font-bold">{value}</div>
        {change !== null && (
          <p className={`text-xs ${change > 0 ? 'text-green-500' : change < 0 ? 'text-red-500' : 'text-muted-foreground'}`}>
            {change > 0 ? '↑' : change < 0 ? '↓' : '→'} {Math.abs(change).toFixed(1)}% change
          </p>
        )}
      </CardContent>
    </Card>
  );
}

function StatusBadge({ count, status }: { count: number, status: "OK" | "WARNING" | "ERROR" }) {
  let variant: "default" | "outline" | "secondary" | "destructive" = "default";
  
  if (status === "WARNING") variant = "secondary";
  if (status === "ERROR") variant = "destructive";
  
  return (
    <div className="flex flex-col items-center gap-1">
      <Badge variant={variant} className="py-1 px-3">
        {status} <span className="ml-1 font-bold">{count}</span>
      </Badge>
    </div>
  );
}

function calculateInsights(telemetry: TelemetryRecord[]) {
  if (!telemetry.length) {
    return {
      averageBattery: null,
      averageTemperature: null,
      batteryTrend: null,
      temperatureTrend: null,
      uptime: "N/A",
      statusCounts: { OK: 0, WARNING: 0, ERROR: 0 },
      statusSummary: "No data available"
    };
  }

  // Count status occurrences
  const statusCounts = {
    OK: telemetry.filter(t => t.status === "OK").length,
    WARNING: telemetry.filter(t => t.status === "WARNING").length,
    ERROR: telemetry.filter(t => t.status === "ERROR").length
  };

  // Calculate battery and temperature stats
  const validBatteryReadings = telemetry.filter(t => t.batteryLevel !== null);
  const validTempReadings = telemetry.filter(t => t.temperature !== null);
  
  const averageBattery = validBatteryReadings.length > 0
    ? Math.round(validBatteryReadings.reduce((sum, t) => sum + (t.batteryLevel || 0), 0) / validBatteryReadings.length)
    : null;
  
  const averageTemperature = validTempReadings.length > 0
    ? Math.round(validTempReadings.reduce((sum, t) => sum + (t.temperature || 0), 0) / validTempReadings.length * 10) / 10
    : null;

  // Calculate trends (from first 10% to last 10% of readings)
  let batteryTrend = null;
  let temperatureTrend = null;
  
  if (validBatteryReadings.length >= 10) {
    const segment = Math.max(Math.floor(validBatteryReadings.length * 0.1), 1);
    const oldAvg = validBatteryReadings.slice(0, segment).reduce((sum, t) => sum + (t.batteryLevel || 0), 0) / segment;
    const newAvg = validBatteryReadings.slice(-segment).reduce((sum, t) => sum + (t.batteryLevel || 0), 0) / segment;
    batteryTrend = oldAvg !== 0 ? ((newAvg - oldAvg) / oldAvg) * 100 : 0;
  }
  
  if (validTempReadings.length >= 10) {
    const segment = Math.max(Math.floor(validTempReadings.length * 0.1), 1);
    const oldAvg = validTempReadings.slice(0, segment).reduce((sum, t) => sum + (t.temperature || 0), 0) / segment;
    const newAvg = validTempReadings.slice(-segment).reduce((sum, t) => sum + (t.temperature || 0), 0) / segment;
    temperatureTrend = oldAvg !== 0 ? ((newAvg - oldAvg) / oldAvg) * 100 : 0;
  }

  // Calculate uptime
  let uptime = "N/A";
  if (telemetry.length > 0) {
    const errorPercentage = (statusCounts.ERROR / telemetry.length) * 100;
    const uptimePercentage = 100 - errorPercentage;
    uptime = `${uptimePercentage.toFixed(1)}%`;
  }

  // Create status summary
  let statusSummary = "No data available";
  if (telemetry.length > 0) {
    const totalReadings = telemetry.length;
    const okPercentage = (statusCounts.OK / totalReadings) * 100;
    
    if (statusCounts.ERROR > 0) {
      statusSummary = `${statusCounts.ERROR} error events detected (${(statusCounts.ERROR / totalReadings * 100).toFixed(1)}% of readings)`;
    } else if (statusCounts.WARNING > 0) {
      statusSummary = `${statusCounts.WARNING} warnings but no errors (${(statusCounts.WARNING / totalReadings * 100).toFixed(1)}% of readings)`;
    } else {
      statusSummary = `Robot operating normally (100% uptime)`;
    }
  }

  return {
    averageBattery,
    averageTemperature,
    batteryTrend,
    temperatureTrend,
    uptime,
    statusCounts,
    statusSummary
  };
}
