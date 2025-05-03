
import { useState, useEffect } from "react";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, Legend } from "recharts";
import { supabase } from "@/integrations/supabase/client";
import { useAuth } from "@/context/AuthContext";
import { Robot, UserProfile } from "@/types/robot";
import { Skeleton } from "@/components/ui/skeleton";
import { format, subDays } from "date-fns";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { Button } from "@/components/ui/button";
import { Download } from "lucide-react";

interface TelemetryHistoryProps {
  robot: Robot;
  userProfile?: UserProfile;
}

export function TelemetryHistory({ robot, userProfile }: TelemetryHistoryProps) {
  const [activeTab, setActiveTab] = useState("battery");
  const [telemetryData, setTelemetryData] = useState<any[]>([]);
  const [loading, setLoading] = useState(true);
  const [timeRange, setTimeRange] = useState("24h");
  const { session } = useAuth();

  useEffect(() => {
    if (!robot?.id || !session) return;
    
    const fetchTelemetryData = async () => {
      setLoading(true);
      
      // Calculate the date range based on the selected time range
      let startDate;
      switch (timeRange) {
        case "7d":
          startDate = subDays(new Date(), 7);
          break;
        case "30d":
          startDate = subDays(new Date(), 30);
          break;
        case "24h":
        default:
          startDate = subDays(new Date(), 1);
          break;
      }
      
      try {
        const { data, error } = await supabase
          .from('telemetry')
          .select('*')
          .eq('robot_id', robot.id)
          .gte('created_at', startDate.toISOString())
          .order('created_at', { ascending: true });
          
        if (error) throw error;
        
        // Process the data for the chart
        const processedData = data.map(item => ({
          timestamp: format(new Date(item.created_at), 'HH:mm MM/dd'),
          rawTimestamp: item.created_at,
          battery: item.battery_level,
          temperature: item.temperature,
          // Add any custom telemetry fields from motor_status if available
          ...(item.motor_status ? 
            typeof item.motor_status === 'string' 
              ? JSON.parse(item.motor_status) 
              : item.motor_status 
            : {})
        }));
        
        setTelemetryData(processedData);
      } catch (error) {
        console.error("Error fetching telemetry data:", error);
      } finally {
        setLoading(false);
      }
    };
    
    fetchTelemetryData();
    
    // Set up a polling interval to refresh data
    const interval = setInterval(fetchTelemetryData, 60000); // Refresh every minute
    
    return () => clearInterval(interval);
  }, [robot.id, session, timeRange]);
  
  const handleDownloadCSV = () => {
    if (!telemetryData.length) return;
    
    // Get all unique keys from the telemetry data
    const allKeys = new Set<string>();
    telemetryData.forEach(item => {
      Object.keys(item).forEach(key => allKeys.add(key));
    });
    
    // Create CSV header
    const keys = Array.from(allKeys);
    let csv = keys.join(',') + '\n';
    
    // Add data rows
    telemetryData.forEach(item => {
      const row = keys.map(key => {
        const value = item[key] !== undefined ? item[key] : '';
        // Handle commas in values by quoting
        return typeof value === 'string' && value.includes(',') 
          ? `"${value}"` 
          : value;
      }).join(',');
      csv += row + '\n';
    });
    
    // Create and trigger download
    const blob = new Blob([csv], { type: 'text/csv' });
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.setAttribute('hidden', '');
    a.setAttribute('href', url);
    a.setAttribute('download', `${robot.name}-telemetry-${format(new Date(), 'yyyy-MM-dd')}.csv`);
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
  };
  
  // Get available metrics from the telemetry data
  const getAvailableMetrics = () => {
    if (!telemetryData.length) return [];
    
    const metrics = new Set<string>();
    telemetryData.forEach(item => {
      Object.keys(item).forEach(key => {
        // Only include numeric values and exclude timestamp fields
        if (
          typeof item[key] === 'number' && 
          key !== 'timestamp' && 
          key !== 'rawTimestamp'
        ) {
          metrics.add(key);
        }
      });
    });
    
    return Array.from(metrics);
  };
  
  const metrics = getAvailableMetrics();
  
  // Configuration for data retention
  const config = {
    retentionDays: userProfile?.telemetry_retention_days ?? 7,
  };

  return (
    <Card className="col-span-3">
      <CardHeader className="flex flex-row items-center justify-between">
        <div>
          <CardTitle>Telemetry History</CardTitle>
          <CardDescription>
            Historical telemetry data for {robot.name}
            {config.retentionDays && (
              <span className="ml-2 text-xs text-muted-foreground">
                (Data retained for {config.retentionDays} days)
              </span>
            )}
          </CardDescription>
        </div>
        <div className="flex items-center gap-2">
          <Select value={timeRange} onValueChange={setTimeRange}>
            <SelectTrigger className="w-[120px]">
              <SelectValue placeholder="Time Range" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="24h">Last 24 Hours</SelectItem>
              <SelectItem value="7d">Last 7 Days</SelectItem>
              <SelectItem value="30d">Last 30 Days</SelectItem>
            </SelectContent>
          </Select>
          
          <Button 
            variant="outline" 
            size="icon" 
            onClick={handleDownloadCSV}
            disabled={!telemetryData.length}
            title="Download CSV"
          >
            <Download className="h-4 w-4" />
          </Button>
        </div>
      </CardHeader>
      <CardContent>
        {loading ? (
          <div className="space-y-2">
            <Skeleton className="h-[300px] w-full" />
          </div>
        ) : telemetryData.length === 0 ? (
          <div className="flex items-center justify-center h-[300px] border rounded-md">
            <p className="text-muted-foreground">No telemetry data available for this time range</p>
          </div>
        ) : (
          <Tabs value={activeTab} onValueChange={setActiveTab}>
            <TabsList className="mb-4">
              {metrics.map(metric => (
                <TabsTrigger key={metric} value={metric}>
                  {metric.charAt(0).toUpperCase() + metric.slice(1)}
                </TabsTrigger>
              ))}
            </TabsList>
            
            {metrics.map(metric => (
              <TabsContent key={metric} value={metric} className="h-[300px]">
                <ResponsiveContainer width="100%" height="100%">
                  <LineChart
                    data={telemetryData}
                    margin={{ top: 5, right: 30, left: 20, bottom: 5 }}
                  >
                    <CartesianGrid strokeDasharray="3 3" />
                    <XAxis 
                      dataKey="timestamp" 
                      tick={{ fontSize: 12 }}
                      interval="preserveStartEnd"
                    />
                    <YAxis />
                    <Tooltip 
                      formatter={(value: any) => [`${value}`, metric.charAt(0).toUpperCase() + metric.slice(1)]}
                      labelFormatter={(label) => `Time: ${label}`}
                    />
                    <Legend />
                    <Line
                      type="monotone"
                      dataKey={metric}
                      stroke="#8884d8"
                      activeDot={{ r: 8 }}
                      name={metric.charAt(0).toUpperCase() + metric.slice(1)}
                    />
                  </LineChart>
                </ResponsiveContainer>
              </TabsContent>
            ))}
          </Tabs>
        )}
      </CardContent>
    </Card>
  );
}
