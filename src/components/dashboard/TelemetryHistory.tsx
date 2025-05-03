
import React, { useState, useEffect } from "react";
import { useQuery } from "@tanstack/react-query";
import { supabase } from "@/integrations/supabase/client";
import { Card, CardContent } from "@/components/ui/card";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { format } from "date-fns";
import { AlertCircle, Battery, Thermometer, MapPin } from "lucide-react";
import { Skeleton } from "@/components/ui/skeleton";
import { Alert, AlertDescription } from "@/components/ui/alert";
import { useAuth } from "@/context/AuthContext";
import { ScrollArea } from "@/components/ui/scroll-area";

interface TelemetryHistoryProps {
  robotId: string;
  retentionDays: number;
}

interface TelemetryRecord {
  id: string;
  battery_level: number | null;
  temperature: number | null;
  location: { latitude: number; longitude: number } | null;
  motor_status: Record<string, any> | null;
  error_codes: string[] | null;
  created_at: string;
}

export function TelemetryHistory({ robotId, retentionDays }: TelemetryHistoryProps) {
  const [limit, setLimit] = useState<number>(50);
  const [page, setPage] = useState<number>(1);
  const [customFieldFilter, setCustomFieldFilter] = useState<string>("");
  const { user } = useAuth();

  // Fetch telemetry history with custom filtering
  const { data: telemetry, isLoading, error } = useQuery({
    queryKey: ['telemetry-history', robotId, limit, page, retentionDays],
    queryFn: async () => {
      // Calculate the date cutoff based on retention days
      const cutoffDate = new Date();
      cutoffDate.setDate(cutoffDate.getDate() - retentionDays);
      
      const { data, error } = await supabase
        .from('telemetry')
        .select('*')
        .eq('robot_id', robotId)
        .gte('created_at', cutoffDate.toISOString())
        .order('created_at', { ascending: false })
        .range((page - 1) * limit, page * limit - 1);

      if (error) throw new Error(error.message);
      
      // Transform data to ensure location has the correct type
      return (data || []).map(item => {
        // Convert location from Json to the expected shape
        let locationData = null;
        if (item.location) {
          // Handle both object and string format
          const loc = typeof item.location === 'string' 
            ? JSON.parse(item.location) 
            : item.location;
            
          locationData = {
            latitude: loc.latitude || (loc.lat || 0),
            longitude: loc.longitude || (loc.lng || 0)
          };
        }
        
        return {
          ...item,
          location: locationData
        };
      }) as TelemetryRecord[];
    },
    enabled: !!robotId && !!user,
  });

  // Update retention days for the user profile
  const handleRetentionChange = async (days: number) => {
    if (!user?.id) return;
    
    const { error } = await supabase
      .from('profiles')
      .update({ telemetry_retention_days: days })
      .eq('id', user.id);
      
    if (error) {
      console.error('Error updating retention days:', error);
    }
  };

  // Custom fields extraction from motor_status
  const getCustomFields = (record: TelemetryRecord): Record<string, any> => {
    if (!record.motor_status) return {};
    
    try {
      return typeof record.motor_status === 'string' 
        ? JSON.parse(record.motor_status) 
        : record.motor_status;
    } catch (e) {
      console.error('Error parsing custom telemetry:', e);
      return {};
    }
  };

  // Filter telemetry records based on custom field filter
  const filteredTelemetry = React.useMemo(() => {
    if (!telemetry) return [];
    if (!customFieldFilter) return telemetry;
    
    return telemetry.filter(record => {
      const customFields = getCustomFields(record);
      return Object.keys(customFields).some(key => 
        key.toLowerCase().includes(customFieldFilter.toLowerCase()) ||
        String(customFields[key]).toLowerCase().includes(customFieldFilter.toLowerCase())
      );
    });
  }, [telemetry, customFieldFilter]);

  // Format date for display
  const formatDateTime = (dateString: string) => {
    try {
      return format(new Date(dateString), "MMM d, yyyy h:mm:ss a");
    } catch (e) {
      return "Invalid date";
    }
  };

  if (isLoading) {
    return (
      <div className="space-y-4">
        <div className="flex items-center justify-between">
          <Skeleton className="h-10 w-40" />
          <Skeleton className="h-10 w-32" />
        </div>
        <Skeleton className="h-[400px] w-full" />
      </div>
    );
  }

  if (error) {
    return (
      <Alert variant="destructive">
        <AlertCircle className="h-4 w-4" />
        <AlertDescription>Failed to load telemetry history: {String(error)}</AlertDescription>
      </Alert>
    );
  }

  return (
    <div className="space-y-4">
      <div className="flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4 mb-4">
        <div className="flex flex-col gap-2">
          <h3 className="text-lg font-medium">Telemetry History</h3>
          <p className="text-sm text-muted-foreground">
            Showing the last {limit} entries within {retentionDays} days
          </p>
        </div>
        
        <div className="flex flex-col sm:flex-row gap-2 w-full sm:w-auto">
          <div className="flex items-center gap-2">
            <span className="text-sm whitespace-nowrap">Records:</span>
            <Select value={limit.toString()} onValueChange={(value) => setLimit(parseInt(value))}>
              <SelectTrigger className="w-20">
                <SelectValue placeholder="50" />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="10">10</SelectItem>
                <SelectItem value="25">25</SelectItem>
                <SelectItem value="50">50</SelectItem>
                <SelectItem value="100">100</SelectItem>
              </SelectContent>
            </Select>
          </div>
          
          <div className="flex items-center gap-2">
            <span className="text-sm whitespace-nowrap">Retention:</span>
            <Select 
              value={retentionDays.toString()} 
              onValueChange={(value) => handleRetentionChange(parseInt(value))}
            >
              <SelectTrigger className="w-20">
                <SelectValue placeholder="7" />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="1">1 day</SelectItem>
                <SelectItem value="7">7 days</SelectItem>
                <SelectItem value="14">14 days</SelectItem>
                <SelectItem value="30">30 days</SelectItem>
                <SelectItem value="90">90 days</SelectItem>
              </SelectContent>
            </Select>
          </div>
          
          <Input
            placeholder="Filter custom fields"
            value={customFieldFilter}
            onChange={(e) => setCustomFieldFilter(e.target.value)}
            className="w-full sm:w-auto"
          />
        </div>
      </div>

      <Card>
        <CardContent className="p-0">
          <ScrollArea className="h-[400px] w-full">
            <Table>
              <TableHeader>
                <TableRow>
                  <TableHead>Time</TableHead>
                  <TableHead>Battery</TableHead>
                  <TableHead>Temp</TableHead>
                  <TableHead>Location</TableHead>
                  <TableHead>Custom Telemetry</TableHead>
                  <TableHead>Errors</TableHead>
                </TableRow>
              </TableHeader>
              <TableBody>
                {filteredTelemetry.length > 0 ? (
                  filteredTelemetry.map((record) => (
                    <TableRow key={record.id}>
                      <TableCell className="font-mono text-xs">
                        {formatDateTime(record.created_at)}
                      </TableCell>
                      <TableCell>
                        {record.battery_level !== null ? (
                          <span className="flex items-center gap-1">
                            <Battery size={14} className={record.battery_level < 20 ? 'text-red-500' : 'text-green-500'} />
                            {record.battery_level}%
                          </span>
                        ) : "N/A"}
                      </TableCell>
                      <TableCell>
                        {record.temperature !== null ? (
                          <span className="flex items-center gap-1">
                            <Thermometer size={14} className={record.temperature > 35 ? 'text-red-500' : 'text-green-500'} />
                            {record.temperature}°C
                          </span>
                        ) : "N/A"}
                      </TableCell>
                      <TableCell>
                        {record.location ? (
                          <span className="flex items-center gap-1">
                            <MapPin size={14} className="text-blue-500" />
                            {record.location.latitude.toFixed(3)}, {record.location.longitude.toFixed(3)}
                          </span>
                        ) : "N/A"}
                      </TableCell>
                      <TableCell>
                        <pre className="text-xs bg-muted p-1 rounded max-w-[200px] overflow-auto">
                          {JSON.stringify(getCustomFields(record), null, 1)}
                        </pre>
                      </TableCell>
                      <TableCell>
                        {record.error_codes && record.error_codes.length > 0 ? (
                          <span className="px-2 py-1 rounded bg-red-100 dark:bg-red-900/20 text-red-800 dark:text-red-300 text-xs">
                            {record.error_codes.join(", ")}
                          </span>
                        ) : (
                          <span className="text-green-600 dark:text-green-400">None</span>
                        )}
                      </TableCell>
                    </TableRow>
                  ))
                ) : (
                  <TableRow>
                    <TableCell colSpan={6} className="text-center py-8 text-muted-foreground">
                      No telemetry data available for the selected time period
                    </TableCell>
                  </TableRow>
                )}
              </TableBody>
            </Table>
          </ScrollArea>
        </CardContent>
      </Card>

      {filteredTelemetry.length > 0 && (
        <div className="flex justify-between items-center mt-4">
          <Button 
            variant="outline" 
            onClick={() => setPage(p => Math.max(1, p - 1))}
            disabled={page === 1}
          >
            Previous
          </Button>
          <span className="text-sm">Page {page}</span>
          <Button 
            variant="outline" 
            onClick={() => setPage(p => p + 1)}
            disabled={filteredTelemetry.length < limit}
          >
            Next
          </Button>
        </div>
      )}
    </div>
  );
}
