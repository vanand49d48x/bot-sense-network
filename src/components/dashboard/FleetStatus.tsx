
import { Robot } from "@/types/robot";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Progress } from "@/components/ui/progress";
import { Doughnut } from "react-chartjs-2";
import { Chart as ChartJS, ArcElement, Tooltip, Legend } from "chart.js";
import { Battery, Thermometer } from "lucide-react";
import { useState, useMemo } from "react";

// Register ChartJS components
ChartJS.register(ArcElement, Tooltip, Legend);

interface FleetStatusProps {
  robots: Robot[];
}

export function FleetStatus({ robots }: FleetStatusProps) {
  const [chartType, setChartType] = useState<'status' | 'battery'>('status');
  
  const statusData = useMemo(() => {
    const online = robots.filter(r => r.status === 'online').length;
    const warning = robots.filter(r => r.status === 'warning').length;
    const offline = robots.filter(r => r.status === 'offline').length;
    
    return {
      labels: ['Online', 'Warning', 'Offline'],
      datasets: [
        {
          data: [online, warning, offline],
          backgroundColor: [
            'rgba(16, 185, 129, 0.8)',  // online - green
            'rgba(245, 158, 11, 0.8)',  // warning - yellow
            'rgba(239, 68, 68, 0.8)',   // offline - red
          ],
          borderColor: [
            'rgba(16, 185, 129, 1)',
            'rgba(245, 158, 11, 1)',
            'rgba(239, 68, 68, 1)',
          ],
          borderWidth: 1,
        },
      ],
    };
  }, [robots]);
  
  const batteryData = useMemo(() => {
    const critical = robots.filter(r => r.batteryLevel < 20).length;
    const low = robots.filter(r => r.batteryLevel >= 20 && r.batteryLevel < 50).length;
    const medium = robots.filter(r => r.batteryLevel >= 50 && r.batteryLevel < 80).length;
    const high = robots.filter(r => r.batteryLevel >= 80).length;
    
    return {
      labels: ['Critical (<20%)', 'Low (20-49%)', 'Medium (50-79%)', 'High (80-100%)'],
      datasets: [
        {
          data: [critical, low, medium, high],
          backgroundColor: [
            'rgba(239, 68, 68, 0.8)',   // critical - red
            'rgba(245, 158, 11, 0.8)',  // low - yellow
            'rgba(59, 130, 246, 0.8)',  // medium - blue
            'rgba(16, 185, 129, 0.8)',  // high - green
          ],
          borderColor: [
            'rgba(239, 68, 68, 1)',
            'rgba(245, 158, 11, 1)',
            'rgba(59, 130, 246, 1)',
            'rgba(16, 185, 129, 1)',
          ],
          borderWidth: 1,
        },
      ],
    };
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

  const chartOptions = {
    responsive: true,
    maintainAspectRatio: false,
    plugins: {
      legend: {
        position: 'right' as const,
        labels: {
          boxWidth: 15,
          padding: 15,
        },
      },
      tooltip: {
        callbacks: {
          label: function(context: any) {
            const label = context.label || '';
            const value = context.raw || 0;
            const total = context.dataset.data.reduce((a: number, b: number) => a + b, 0);
            const percentage = total ? Math.round((value / total) * 100) : 0;
            return `${label}: ${value} (${percentage}%)`;
          }
        }
      }
    },
    cutout: '60%',
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
            <Doughnut 
              data={chartType === 'status' ? statusData : batteryData} 
              options={chartOptions} 
            />
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
