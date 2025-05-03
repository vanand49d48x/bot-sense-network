
import React, { useState, useMemo } from "react";
import { useQuery } from "@tanstack/react-query";
import { supabase } from "@/integrations/supabase/client";
import { format, subDays } from "date-fns";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { Label } from "@/components/ui/label";
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, Legend } from "recharts";
import {
  ChartContainer,
  ChartTooltipContent,
  ChartLegendContent,
} from "@/components/ui/chart";
import { Battery, Thermometer, ActivitySquare } from "lucide-react";
import { Skeleton } from "@/components/ui/skeleton";
import { Alert, AlertDescription } from "@/components/ui/alert";
import { AlertCircle } from "lucide-react";

interface TelemetryChartProps {
  robotId: string;
  retentionDays: number;
}

interface TelemetryRecord {
  id: string;
  battery_level: number | null;
  temperature: number | null;
  created_at: string;
  motor_status: Record<string, any> | null;
}

type MetricType = "battery_level" | "temperature" | "custom";
type TimeRange = "24h" | "7d" | "30d" | "all";

export function TelemetryChart({ robotId, retentionDays }: TelemetryChartProps) {
  const [metricType, setMetricType] = useState<MetricType>("battery_level");
  const [timeRange, setTimeRange] = useState<TimeRange>("24h");
  const [customMetric, setCustomMetric] = useState<string>("");
  
  // Calculate date range based on selected time range
  const getDateRange = () => {
    const now = new Date();
    switch (timeRange) {
      case "24h":
        return subDays(now, 1).toISOString();
      case "7d":
        return subDays(now, 7).toISOString();
      case "30d":
        return subDays(now, 30).toISOString();
      case "all":
        return subDays(now, retentionDays).toISOString();
      default:
        return subDays(now, 1).toISOString();
    }
  };
  
  // Fetch telemetry data for the selected time range
  const { data: telemetry, isLoading, error } = useQuery({
    queryKey: ['telemetry-chart', robotId, timeRange, retentionDays],
    queryFn: async () => {
      const cutoffDate = getDateRange();
      
      const { data, error } = await supabase
        .from('telemetry')
        .select('*')
        .eq('robot_id', robotId)
        .gte('created_at', cutoffDate)
        .order('created_at', { ascending: true });

      if (error) throw new Error(error.message);
      
      return data as TelemetryRecord[];
    },
    enabled: !!robotId,
  });

  // Extract available custom metrics from the first telemetry record with motor_status
  const availableCustomMetrics = useMemo(() => {
    if (!telemetry || telemetry.length === 0) return [];
    
    const metricsRecord = telemetry.find(record => record.motor_status !== null);
    if (!metricsRecord || !metricsRecord.motor_status) return [];
    
    return Object.keys(metricsRecord.motor_status);
  }, [telemetry]);

  // Set first available custom metric when they become available and none is selected
  React.useEffect(() => {
    if (availableCustomMetrics.length > 0 && customMetric === "") {
      setCustomMetric(availableCustomMetrics[0]);
    }
  }, [availableCustomMetrics, customMetric]);

  // Transform data for recharts
  const chartData = useMemo(() => {
    if (!telemetry || telemetry.length === 0) return [];
    
    return telemetry.map(record => {
      let metricValue = null;
      
      if (metricType === "battery_level") {
        metricValue = record.battery_level;
      } else if (metricType === "temperature") {
        metricValue = record.temperature;
      } else if (metricType === "custom" && record.motor_status && customMetric) {
        metricValue = record.motor_status[customMetric];
      }
      
      return {
        time: format(new Date(record.created_at), "MM/dd HH:mm"),
        value: metricValue,
        timestamp: record.created_at
      };
    });
  }, [telemetry, metricType, customMetric]);

  const getMetricName = () => {
    switch (metricType) {
      case "battery_level":
        return "Battery Level (%)";
      case "temperature":
        return "Temperature (°C)";
      case "custom":
        return customMetric;
      default:
        return "Value";
    }
  };

  const getMetricIcon = () => {
    switch (metricType) {
      case "battery_level":
        return <Battery className="h-5 w-5" />;
      case "temperature":
        return <Thermometer className="h-5 w-5" />;
      default:
        return <ActivitySquare className="h-5 w-5" />;
    }
  };

  if (isLoading) {
    return (
      <div className="space-y-4">
        <div className="flex justify-between">
          <Skeleton className="h-10 w-48" />
          <Skeleton className="h-10 w-28" />
        </div>
        <Skeleton className="h-[300px] w-full" />
      </div>
    );
  }

  if (error) {
    return (
      <Alert variant="destructive">
        <AlertCircle className="h-4 w-4" />
        <AlertDescription>Failed to load telemetry data: {String(error)}</AlertDescription>
      </Alert>
    );
  }

  return (
    <Card>
      <CardHeader className="pb-2">
        <div className="flex flex-col sm:flex-row sm:items-center justify-between gap-2">
          <CardTitle className="flex items-center gap-2">
            {getMetricIcon()}
            <span>{getMetricName()}</span>
          </CardTitle>
          <div className="flex flex-wrap gap-2">
            <div className="flex items-center gap-2">
              <Label htmlFor="metric-type">Metric:</Label>
              <Select value={metricType} onValueChange={(value) => setMetricType(value as MetricType)}>
                <SelectTrigger className="w-[180px]" id="metric-type">
                  <SelectValue placeholder="Select metric" />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="battery_level">Battery Level</SelectItem>
                  <SelectItem value="temperature">Temperature</SelectItem>
                  {availableCustomMetrics.length > 0 && (
                    <SelectItem value="custom">Custom Metric</SelectItem>
                  )}
                </SelectContent>
              </Select>
            </div>
            
            {metricType === "custom" && availableCustomMetrics.length > 0 && (
              <div className="flex items-center gap-2">
                <Label htmlFor="custom-metric">Custom:</Label>
                <Select value={customMetric} onValueChange={setCustomMetric}>
                  <SelectTrigger className="w-[180px]" id="custom-metric">
                    <SelectValue placeholder="Select custom metric" />
                  </SelectTrigger>
                  <SelectContent>
                    {availableCustomMetrics.map(metric => (
                      <SelectItem key={metric} value={metric}>{metric}</SelectItem>
                    ))}
                  </SelectContent>
                </Select>
              </div>
            )}
            
            <div className="flex items-center gap-2">
              <Label htmlFor="time-range">Range:</Label>
              <Select value={timeRange} onValueChange={(value) => setTimeRange(value as TimeRange)}>
                <SelectTrigger className="w-[100px]" id="time-range">
                  <SelectValue placeholder="Time range" />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="24h">24h</SelectItem>
                  <SelectItem value="7d">7 days</SelectItem>
                  <SelectItem value="30d">30 days</SelectItem>
                  <SelectItem value="all">All</SelectItem>
                </SelectContent>
              </Select>
            </div>
          </div>
        </div>
      </CardHeader>
      <CardContent>
        {chartData.length > 0 ? (
          <div className="h-[300px] w-full">
            <ChartContainer config={{
              metric: {
                label: getMetricName(),
                theme: {
                  light: "#2563eb",
                  dark: "#3b82f6"
                }
              }
            }}>
              <ResponsiveContainer width="100%" height="100%">
                <LineChart data={chartData} margin={{ top: 5, right: 5, left: 5, bottom: 5 }}>
                  <CartesianGrid strokeDasharray="3 3" />
                  <XAxis 
                    dataKey="time" 
                    tick={{ fontSize: 12 }}
                    tickCount={7}
                  />
                  <YAxis 
                    tick={{ fontSize: 12 }} 
                    domain={[(dataMin: number) => Math.max(0, dataMin * 0.9), (dataMax: number) => dataMax * 1.1]}
                  />
                  <Tooltip content={<ChartTooltipContent formatter={(value) => [`${value}`, getMetricName()]} />} />
                  <Legend content={<ChartLegendContent />} />
                  <Line 
                    type="monotone" 
                    dataKey="value" 
                    stroke="var(--color-metric)" 
                    strokeWidth={2} 
                    dot={{ r: 3 }} 
                    activeDot={{ r: 5 }} 
                    name={getMetricName()} 
                  />
                </LineChart>
              </ResponsiveContainer>
            </ChartContainer>
          </div>
        ) : (
          <div className="flex items-center justify-center h-[300px] text-muted-foreground">
            No telemetry data available for the selected time period
          </div>
        )}
      </CardContent>
    </Card>
  );
}
