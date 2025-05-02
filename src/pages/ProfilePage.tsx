
import { useState, useEffect } from "react";
import { MainLayout } from "@/components/layout/MainLayout";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { useAuth } from "@/context/AuthContext";
import { toast } from "@/components/ui/sonner";
import { PlusCircle, Loader2, X, BellRing, Thermometer, Battery, AlertTriangle } from "lucide-react";
import { supabase } from "@/integrations/supabase/client";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { Separator } from "@/components/ui/separator";

type AlertType = "battery" | "temperature" | "offline" | "error";
type AlertThreshold = {
  type: AlertType;
  threshold: number;
  enabled: boolean;
};

export default function ProfilePage() {
  const { user } = useAuth();
  const [customRobotTypes, setCustomRobotTypes] = useState<string[]>([]);
  const [customAlerts, setCustomAlerts] = useState<AlertThreshold[]>([]);
  const [newType, setNewType] = useState("");
  const [alertType, setAlertType] = useState<AlertType>("battery");
  const [alertThreshold, setAlertThreshold] = useState<number>(20);
  const [isLoading, setIsLoading] = useState(false);
  
  useEffect(() => {
    if (user) {
      fetchCustomRobotTypes();
      fetchCustomAlerts();
    }
  }, [user]);
  
  const fetchCustomRobotTypes = async () => {
    try {
      if (!user) return;
      
      setIsLoading(true);
      const { data: profileData, error } = await supabase
        .from('profiles')
        .select('custom_robot_types')
        .eq('id', user.id)
        .single();
        
      if (error) throw error;
      
      if (profileData?.custom_robot_types) {
        setCustomRobotTypes(profileData.custom_robot_types);
      } else {
        setCustomRobotTypes([]);
      }
    } catch (error: any) {
      console.error('Error fetching custom robot types:', error.message);
      toast.error('Failed to load custom robot types');
    } finally {
      setIsLoading(false);
    }
  };

  const fetchCustomAlerts = async () => {
    try {
      if (!user) return;
      
      setIsLoading(true);
      const { data: profileData, error } = await supabase
        .from('profiles')
        .select('custom_alerts')
        .eq('id', user.id)
        .single();
        
      if (error) throw error;
      
      if (profileData?.custom_alerts) {
        setCustomAlerts(profileData.custom_alerts);
      } else {
        // Set default alerts if none exist
        setCustomAlerts([
          { type: "battery", threshold: 20, enabled: true },
          { type: "temperature", threshold: 35, enabled: true },
          { type: "offline", threshold: 0, enabled: true },
          { type: "error", threshold: 0, enabled: true }
        ]);
      }
    } catch (error: any) {
      console.error('Error fetching custom alerts:', error.message);
      toast.error('Failed to load custom alerts');
      
      // Set default alerts on error
      setCustomAlerts([
        { type: "battery", threshold: 20, enabled: true },
        { type: "temperature", threshold: 35, enabled: true },
        { type: "offline", threshold: 0, enabled: true },
        { type: "error", threshold: 0, enabled: true }
      ]);
    } finally {
      setIsLoading(false);
    }
  };
  
  const addCustomRobotType = async () => {
    if (!newType.trim()) {
      toast.error('Please enter a valid robot type');
      return;
    }
    
    try {
      setIsLoading(true);
      
      // Check if type already exists
      if (customRobotTypes.includes(newType)) {
        toast.error('This robot type already exists');
        return;
      }
      
      const updatedTypes = [...customRobotTypes, newType.trim()];
      
      const { error } = await supabase
        .from('profiles')
        .update({ custom_robot_types: updatedTypes })
        .eq('id', user?.id);
        
      if (error) throw error;
      
      setCustomRobotTypes(updatedTypes);
      setNewType("");
      
      toast.success('Robot type added successfully');
    } catch (error: any) {
      console.error('Error adding robot type:', error.message);
      toast.error('Failed to add robot type');
    } finally {
      setIsLoading(false);
    }
  };
  
  const deleteCustomRobotType = async (typeToDelete: string) => {
    try {
      setIsLoading(true);
      
      const updatedTypes = customRobotTypes.filter(type => type !== typeToDelete);
      
      const { error } = await supabase
        .from('profiles')
        .update({ custom_robot_types: updatedTypes })
        .eq('id', user?.id);
        
      if (error) throw error;
      
      setCustomRobotTypes(updatedTypes);
      toast.success('Robot type removed');
    } catch (error: any) {
      console.error('Error removing robot type:', error.message);
      toast.error('Failed to remove robot type');
    } finally {
      setIsLoading(false);
    }
  };

  const addCustomAlert = async () => {
    try {
      setIsLoading(true);
      
      // Check if alert already exists
      const existingAlertIndex = customAlerts.findIndex(alert => alert.type === alertType);
      let updatedAlerts: AlertThreshold[];
      
      if (existingAlertIndex >= 0) {
        // Update existing alert
        updatedAlerts = [...customAlerts];
        updatedAlerts[existingAlertIndex] = {
          ...updatedAlerts[existingAlertIndex],
          threshold: alertThreshold,
          enabled: true
        };
      } else {
        // Add new alert
        updatedAlerts = [
          ...customAlerts,
          {
            type: alertType,
            threshold: alertThreshold,
            enabled: true
          }
        ];
      }
      
      const { error } = await supabase
        .from('profiles')
        .update({ custom_alerts: updatedAlerts })
        .eq('id', user?.id);
        
      if (error) throw error;
      
      setCustomAlerts(updatedAlerts);
      toast.success('Alert threshold updated');
    } catch (error: any) {
      console.error('Error updating alert threshold:', error.message);
      toast.error('Failed to update alert threshold');
    } finally {
      setIsLoading(false);
    }
  };

  const toggleAlertEnabled = async (type: AlertType) => {
    try {
      const updatedAlerts = customAlerts.map(alert => {
        if (alert.type === type) {
          return { ...alert, enabled: !alert.enabled };
        }
        return alert;
      });
      
      const { error } = await supabase
        .from('profiles')
        .update({ custom_alerts: updatedAlerts })
        .eq('id', user?.id);
        
      if (error) throw error;
      
      setCustomAlerts(updatedAlerts);
      const alert = updatedAlerts.find(a => a.type === type);
      toast.success(`Alert ${alert?.enabled ? 'enabled' : 'disabled'}`);
    } catch (error: any) {
      console.error('Error toggling alert:', error.message);
      toast.error('Failed to update alert settings');
    }
  };

  const getAlertIcon = (type: AlertType) => {
    switch (type) {
      case "battery":
        return <Battery className="h-4 w-4 mr-2" />;
      case "temperature":
        return <Thermometer className="h-4 w-4 mr-2" />;
      case "offline":
        return <BellRing className="h-4 w-4 mr-2" />;
      case "error":
        return <AlertTriangle className="h-4 w-4 mr-2" />;
    }
  };

  const getAlertLabel = (type: AlertType) => {
    switch (type) {
      case "battery":
        return "Low Battery";
      case "temperature":
        return "High Temperature";
      case "offline":
        return "Offline";
      case "error":
        return "Error";
    }
  };

  const getAlertDescription = (alert: AlertThreshold) => {
    switch (alert.type) {
      case "battery":
        return `Alert when battery is below ${alert.threshold}%`;
      case "temperature":
        return `Alert when temperature exceeds ${alert.threshold}°C`;
      case "offline":
        return "Alert when robot goes offline";
      case "error":
        return "Alert when robot reports errors";
    }
  };

  return (
    <MainLayout>
      <div className="container py-6">
        <h1 className="text-3xl font-bold mb-6">Profile Settings</h1>
        
        <div className="grid gap-6 md:grid-cols-2">
          <Card>
            <CardHeader>
              <CardTitle>Custom Robot Types</CardTitle>
              <CardDescription>
                Define your own robot types to use when adding new robots
              </CardDescription>
            </CardHeader>
            <CardContent>
              <div className="space-y-4">
                <div className="flex items-end gap-2">
                  <div className="flex-1">
                    <Label htmlFor="new-type">Add New Robot Type</Label>
                    <Input 
                      id="new-type"
                      placeholder="Enter robot type name" 
                      value={newType}
                      onChange={(e) => setNewType(e.target.value)}
                      disabled={isLoading}
                    />
                  </div>
                  <Button 
                    onClick={addCustomRobotType}
                    disabled={isLoading || !newType.trim()}
                  >
                    {isLoading ? (
                      <Loader2 className="h-4 w-4 animate-spin" />
                    ) : (
                      <PlusCircle className="h-4 w-4 mr-2" />
                    )}
                    Add
                  </Button>
                </div>
                
                {isLoading && <div className="text-center py-4">Loading...</div>}
                
                {!isLoading && customRobotTypes.length === 0 && (
                  <div className="text-center text-muted-foreground py-4">
                    No custom robot types added yet
                  </div>
                )}
                
                <div className="flex flex-wrap gap-2 mt-4">
                  {customRobotTypes.map((type, index) => (
                    <div 
                      key={index} 
                      className="bg-secondary text-secondary-foreground px-3 py-1 rounded-full flex items-center gap-1"
                    >
                      <span>{type}</span>
                      <button 
                        onClick={() => deleteCustomRobotType(type)}
                        className="hover:text-destructive transition-colors"
                        disabled={isLoading}
                        aria-label={`Delete ${type} robot type`}
                      >
                        <X className="h-3 w-3" />
                      </button>
                    </div>
                  ))}
                </div>
              </div>
            </CardContent>
            <CardFooter className="text-sm text-muted-foreground">
              Custom robot types will appear in the dropdown when adding a new robot
            </CardFooter>
          </Card>

          <Card>
            <CardHeader>
              <CardTitle>Custom Alert Settings</CardTitle>
              <CardDescription>
                Configure when you want to receive alerts for your robots
              </CardDescription>
            </CardHeader>
            <CardContent>
              <div className="space-y-4">
                <div className="grid gap-4">
                  <div className="grid grid-cols-2 gap-4">
                    <div>
                      <Label htmlFor="alert-type">Alert Type</Label>
                      <Select 
                        defaultValue={alertType} 
                        onValueChange={(value) => setAlertType(value as AlertType)}
                      >
                        <SelectTrigger id="alert-type">
                          <SelectValue placeholder="Select alert type" />
                        </SelectTrigger>
                        <SelectContent>
                          <SelectItem value="battery">Low Battery</SelectItem>
                          <SelectItem value="temperature">High Temperature</SelectItem>
                          <SelectItem value="offline">Offline Status</SelectItem>
                          <SelectItem value="error">Errors</SelectItem>
                        </SelectContent>
                      </Select>
                    </div>

                    <div>
                      <Label htmlFor="alert-threshold">
                        {alertType === "battery" ? "Battery % Threshold" : 
                         alertType === "temperature" ? "Temperature °C Threshold" : 
                         "Threshold"}
                      </Label>
                      <Input 
                        id="alert-threshold"
                        type="number" 
                        value={alertThreshold}
                        onChange={(e) => setAlertThreshold(Number(e.target.value))}
                        disabled={alertType === "offline" || alertType === "error"}
                        min={alertType === "battery" ? 1 : 0}
                        max={alertType === "battery" ? 100 : 100}
                      />
                    </div>
                  </div>

                  <Button 
                    onClick={addCustomAlert}
                    disabled={isLoading}
                  >
                    {isLoading ? (
                      <Loader2 className="h-4 w-4 animate-spin" />
                    ) : (
                      <PlusCircle className="h-4 w-4 mr-2" />
                    )}
                    Save Alert Setting
                  </Button>
                </div>

                <Separator className="my-4" />

                <h3 className="text-lg font-medium mb-2">Current Alert Settings</h3>
                
                {customAlerts.length === 0 ? (
                  <div className="text-muted-foreground">No custom alerts configured</div>
                ) : (
                  <div className="space-y-3">
                    {customAlerts.map((alert, index) => (
                      <div key={index} className="flex items-center justify-between bg-muted p-3 rounded-md">
                        <div className="flex items-center">
                          {getAlertIcon(alert.type)}
                          <div>
                            <p className="font-medium">{getAlertLabel(alert.type)}</p>
                            <p className="text-sm text-muted-foreground">{getAlertDescription(alert)}</p>
                          </div>
                        </div>
                        <Button 
                          variant={alert.enabled ? "default" : "outline"} 
                          size="sm"
                          onClick={() => toggleAlertEnabled(alert.type)}
                        >
                          {alert.enabled ? "Enabled" : "Disabled"}
                        </Button>
                      </div>
                    ))}
                  </div>
                )}
              </div>
            </CardContent>
          </Card>
        </div>
      </div>
    </MainLayout>
  );
}
