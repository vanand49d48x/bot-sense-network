
import { Robot } from "@/types/robot";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Progress } from "@/components/ui/progress";
import { Battery, Thermometer } from "lucide-react";
import { useState, useMemo } from "react";
import { PieChart, Pie, ResponsiveContainer, Cell, Legend, Tooltip } from "recharts";

interface FleetStatusProps {
  robots: Robot[];
}

export function FleetStatus({ robots }: FleetStatusProps) {
  const [chartType, setChartType] = useState<'status' | 'battery'>('status');
  
  const statusData = useMemo(() => {
    const online = robots.filter(r => r.status === 'online').length;
    const warning = robots.filter(r => r.status === 'warning').length;
    const offline = robots.filter(r => r.status === 'offline').length;
    
    return [
      { name: 'Online', value: online, color: '#10b981' },
      { name: 'Warning', value: warning, color: '#f59e0b' },
      { name: 'Offline', value: offline, color: '#ef4444' }
    ];
  }, [robots]);
  
  const batteryData = useMemo(() => {
    const critical = robots.filter(r => r.batteryLevel < 20).length;
    const low = robots.filter(r => r.batteryLevel >= 20 && r.batteryLevel < 50).length;
    const medium = robots.filter(r => r.batteryLevel >= 50 && r.batteryLevel < 80).length;
    const high = robots.filter(r => r.batteryLevel >= 80).length;
    
    return [
      { name: 'Critical (<20%)', value: critical, color: '#ef4444' },
      { name: 'Low (20-49%)', value: low, color: '#f59e0b' },
      { name: 'Medium (50-79%)', value: medium, color: '#3b82f6' },
      { name: 'High (80-100%)', value: high, color: '#10b981' }
    ];
  }, [robots]);

  const averageBattery = useMemo(() => {
    if (robots.length === 0) return 0;
    const sum = robots.reduce((acc, robot) => acc + robot.batteryLevel, 0);
    return Math.round(sum / robots.length);
  }, [robots]);

  const averageTemperature = useMemo(() => {
    if (robots.length === 0) return 0;
    const sum = robots.reduce((acc, robot) => acc + robot.temperature, 0);
    return Math.round((sum / robots.length) * 10) / 10; // Round to 1 decimal place
  }, [robots]);

  const renderCustomizedLabel = ({ cx, cy, midAngle, innerRadius, outerRadius, percent }: any) => {
    const radius = innerRadius + (outerRadius - innerRadius) * 0.5;
    const x = cx + radius * Math.cos(-midAngle * Math.PI / 180);
    const y = cy + radius * Math.sin(-midAngle * Math.PI / 180);

    return (
      percent > 0.05 ? (
        <text 
          x={x} 
          y={y} 
          fill="white" 
          textAnchor="middle" 
          dominantBaseline="central"
          fontSize={12}
        >
          {`${(percent * 100).toFixed(0)}%`}
        </text>
      ) : null
    );
  };

  return (
    <Card className="mt-6 animate-fade-in">
      <CardHeader className="pb-2">
        <div className="flex items-center justify-between">
          <CardTitle>Fleet Status Overview</CardTitle>
          <div className="flex items-center gap-2">
            <button 
              onClick={() => setChartType('status')}
              className={`px-3 py-1 text-xs rounded-md ${chartType === 'status' 
                ? 'bg-primary text-primary-foreground' 
                : 'bg-muted hover:bg-muted/80'}`}
            >
              Status
            </button>
            <button 
              onClick={() => setChartType('battery')}
              className={`px-3 py-1 text-xs rounded-md ${chartType === 'battery' 
                ? 'bg-primary text-primary-foreground' 
                : 'bg-muted hover:bg-muted/80'}`}
            >
              Battery
            </button>
          </div>
        </div>
      </CardHeader>
      <CardContent>
        <div className="grid md:grid-cols-3 gap-6">
          <div className="md:col-span-1 space-y-4">
            <div className="flex items-center gap-2 mb-2">
              <Battery className="w-5 h-5 text-robot-online" />
              <div className="text-sm">Average Battery Level</div>
            </div>
            <div className="text-2xl font-semibold mb-1">{averageBattery}%</div>
            <Progress value={averageBattery} className="h-2" />
            
            <div className="pt-4">
              <div className="flex items-center gap-2 mb-2">
                <Thermometer className="w-5 h-5 text-robot-warning" />
                <div className="text-sm">Average Temperature</div>
              </div>
              <div className="text-2xl font-semibold">
                {averageTemperature}°C
              </div>
              <div className="text-xs text-muted-foreground">
                {averageTemperature < 30 ? "Normal" : 
                 averageTemperature < 40 ? "Elevated" : "Critical"}
              </div>
            </div>
          </div>
          
          <div className="md:col-span-2 h-64 relative">
            <ResponsiveContainer width="100%" height="100%">
              <PieChart>
                <Pie
                  data={chartType === 'status' ? statusData : batteryData}
                  cx="50%"
                  cy="50%"
                  labelLine={false}
                  label={renderCustomizedLabel}
                  outerRadius={80}
                  innerRadius={40}
                  fill="#8884d8"
                  dataKey="value"
                >
                  {(chartType === 'status' ? statusData : batteryData).map((entry, index) => (
                    <Cell key={`cell-${index}`} fill={entry.color} />
                  ))}
                </Pie>
                <Tooltip 
                  formatter={(value, name) => [`${value} robots`, name]}
                  contentStyle={{ 
                    backgroundColor: 'rgba(23, 23, 23, 0.8)', 
                    borderRadius: '0.375rem',
                    border: '1px solid rgba(63, 63, 70, 0.5)',
                    color: '#fff' 
                  }}
                />
                <Legend 
                  layout="vertical" 
                  verticalAlign="middle" 
                  align="right"
                  wrapperStyle={{ paddingLeft: '10px' }}
                />
              </PieChart>
            </ResponsiveContainer>
          </div>
        </div>
        
        <div className="mt-4 grid grid-cols-2 md:grid-cols-4 gap-2">
          <div className="text-center p-2 bg-muted/30 rounded-md">
            <div className="text-xs text-muted-foreground">Total Fleet</div>
            <div className="text-lg font-medium">{robots.length} Robots</div>
          </div>
          <div className="text-center p-2 bg-muted/30 rounded-md">
            <div className="text-xs text-muted-foreground">Online</div>
            <div className="text-lg font-medium text-robot-online">
              {robots.filter(r => r.status === 'online').length}
            </div>
          </div>
          <div className="text-center p-2 bg-muted/30 rounded-md">
            <div className="text-xs text-muted-foreground">Warning</div>
            <div className="text-lg font-medium text-robot-warning">
              {robots.filter(r => r.status === 'warning').length}
            </div>
          </div>
          <div className="text-center p-2 bg-muted/30 rounded-md">
            <div className="text-xs text-muted-foreground">Offline</div>
            <div className="text-lg font-medium text-robot-offline">
              {robots.filter(r => r.status === 'offline').length}
            </div>
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
