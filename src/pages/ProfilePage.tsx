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
import { Json } from "@/types/supabase";
import { Switch } from "@/components/ui/switch";
import { Collapsible, CollapsibleContent, CollapsibleTrigger } from "@/components/ui/collapsible";
import { Checkbox } from "@/components/ui/checkbox";
import { ToggleGroup, ToggleGroupItem } from "@/components/ui/toggle-group";
import { 
  DropdownMenu,
  DropdownMenuTrigger,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuCheckboxItem,
  DropdownMenuSeparator,
  DropdownMenuLabel
} from "@/components/ui/dropdown-menu";

type AlertType = "battery" | "temperature" | "offline" | "error" | string;

// Define metadata for alert types to better handle thresholds and UI
interface AlertTypeMetadata {
  label: string;
  description: string;
  icon: React.ReactNode;
  hasThreshold: boolean;
  defaultThreshold: number;
  thresholdLabel?: string;
  thresholdUnit?: string;
  thresholdMin?: number;
  thresholdMax?: number;
}

// Condition object that defines a single alert condition
type AlertCondition = {
  type: AlertType;
  threshold: number;
};

// Main alert threshold with multiple conditions
type AlertThreshold = {
  type: AlertType;           // Primary alert type
  threshold: number;         // Primary threshold
  enabled: boolean;          // Is this alert enabled
  andCondition: boolean;     // Use AND or OR logic for additional conditions
  additionalConditions?: AlertCondition[]; // Additional conditions to check
  customTelemetry?: boolean; // Flag for custom telemetry data
};

export default function ProfilePage() {
  const { user } = useAuth();
  const [customRobotTypes, setCustomRobotTypes] = useState<string[]>([]);
  const [customAlerts, setCustomAlerts] = useState<AlertThreshold[]>([]);
  const [customTelemetryTypes, setCustomTelemetryTypes] = useState<string[]>([]);
  const [newType, setNewType] = useState("");
  const [newTelemetryType, setNewTelemetryType] = useState("");
  const [alertType, setAlertType] = useState<AlertType>("battery");
  const [alertThreshold, setAlertThreshold] = useState<number>(20);
  const [useAndCondition, setUseAndCondition] = useState<boolean>(false);
  const [isLoading, setIsLoading] = useState(false);
  const [selectedAlert, setSelectedAlert] = useState<AlertThreshold | null>(null);
  const [isAddingCondition, setIsAddingCondition] = useState(false);
  const [additionalConditionType, setAdditionalConditionType] = useState<AlertType>("temperature");
  const [additionalThreshold, setAdditionalThreshold] = useState<number>(35);
  const [isOpenConditions, setIsOpenConditions] = useState<{[key: string]: boolean}>({});
  
  // Define standard alert types metadata
  const alertTypeMetadata: Record<string, AlertTypeMetadata> = {
    "battery": {
      label: "Low Battery",
      description: "Alert when battery level drops below threshold",
      icon: <Battery className="h-4 w-4 mr-2" />,
      hasThreshold: true,
      defaultThreshold: 20,
      thresholdLabel: "Battery Level",
      thresholdUnit: "%",
      thresholdMin: 1,
      thresholdMax: 100
    },
    "temperature": {
      label: "High Temperature",
      description: "Alert when temperature exceeds threshold",
      icon: <Thermometer className="h-4 w-4 mr-2" />,
      hasThreshold: true,
      defaultThreshold: 35,
      thresholdLabel: "Temperature",
      thresholdUnit: "°C",
      thresholdMin: 0,
      thresholdMax: 100
    },
    "offline": {
      label: "Offline Status",
      description: "Alert when robot goes offline",
      icon: <BellRing className="h-4 w-4 mr-2" />,
      hasThreshold: false,
      defaultThreshold: 0
    },
    "error": {
      label: "Error",
      description: "Alert when robot reports errors",
      icon: <AlertTriangle className="h-4 w-4 mr-2" />,
      hasThreshold: false,
      defaultThreshold: 0
    }
  };
  
  useEffect(() => {
    if (user) {
      fetchCustomRobotTypes();
      fetchCustomAlerts();
      fetchCustomTelemetryTypes();
    }
  }, [user]);

  useEffect(() => {
    // Update threshold when alert type changes based on metadata
    if (alertType in alertTypeMetadata) {
      setAlertThreshold(alertTypeMetadata[alertType].defaultThreshold);
    }
  }, [alertType]);
  
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

  const fetchCustomTelemetryTypes = async () => {
    try {
      if (!user) return;
      
      setIsLoading(true);
      const { data: profileData, error } = await supabase
        .from('profiles')
        .select('custom_telemetry_types')
        .eq('id', user.id)
        .single();
        
      if (error) throw error;
      
      if (profileData?.custom_telemetry_types && Array.isArray(profileData.custom_telemetry_types)) {
        setCustomTelemetryTypes(profileData.custom_telemetry_types);
      } else {
        setCustomTelemetryTypes([]);
      }
    } catch (error: any) {
      console.error('Error fetching custom telemetry types:', error.message);
      toast.error('Failed to load custom telemetry types');
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
      
      if (profileData?.custom_alerts && Array.isArray(profileData.custom_alerts)) {
        // Type assertion to properly convert from Json[] to AlertThreshold[]
        const typedAlerts: AlertThreshold[] = profileData.custom_alerts.map((alert: Json) => {
          // Ensure the alert has the right shape before casting
          if (typeof alert === 'object' && alert !== null &&
              'type' in alert && 'threshold' in alert && 'enabled' in alert) {
            return {
              type: alert.type as AlertType,
              threshold: Number(alert.threshold),
              enabled: Boolean(alert.enabled),
              andCondition: 'andCondition' in alert ? Boolean(alert.andCondition) : false,
              additionalConditions: 'additionalConditions' in alert && Array.isArray(alert.additionalConditions)
                ? (alert.additionalConditions as any[]).map(cond => ({
                    type: cond.type as AlertType,
                    threshold: Number(cond.threshold)
                  }))
                : [],
              customTelemetry: 'customTelemetry' in alert ? Boolean(alert.customTelemetry) : false
            };
          }
          // Return a default alert for any invalid data
          return {
            type: 'battery' as AlertType,
            threshold: 20,
            enabled: false,
            andCondition: false,
            additionalConditions: [],
            customTelemetry: false
          };
        });
        
        setCustomAlerts(typedAlerts);
      } else {
        // Set default alerts if none exist
        setCustomAlerts([
          { type: "battery", threshold: 20, enabled: true, andCondition: false, additionalConditions: [] },
          { type: "temperature", threshold: 35, enabled: true, andCondition: false, additionalConditions: [] },
          { type: "offline", threshold: 0, enabled: true, andCondition: false, additionalConditions: [] },
          { type: "error", threshold: 0, enabled: true, andCondition: false, additionalConditions: [] }
        ]);
      }
    } catch (error: any) {
      console.error('Error fetching custom alerts:', error.message);
      toast.error('Failed to load custom alerts');
      
      // Set default alerts on error
      setCustomAlerts([
        { type: "battery", threshold: 20, enabled: true, andCondition: false, additionalConditions: [] },
        { type: "temperature", threshold: 35, enabled: true, andCondition: false, additionalConditions: [] },
        { type: "offline", threshold: 0, enabled: true, andCondition: false, additionalConditions: [] },
        { type: "error", threshold: 0, enabled: true, andCondition: false, additionalConditions: [] }
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

  const addCustomTelemetryType = async () => {
    if (!newTelemetryType.trim()) {
      toast.error('Please enter a valid telemetry type');
      return;
    }
    
    try {
      setIsLoading(true);
      
      // Check if type already exists
      if (customTelemetryTypes.includes(newTelemetryType)) {
        toast.error('This telemetry type already exists');
        return;
      }
      
      const updatedTypes = [...customTelemetryTypes, newTelemetryType.trim()];
      
      const { error } = await supabase
        .from('profiles')
        .update({ custom_telemetry_types: updatedTypes })
        .eq('id', user?.id);
        
      if (error) throw error;
      
      setCustomTelemetryTypes(updatedTypes);
      setNewTelemetryType("");
      
      toast.success('Custom telemetry type added successfully');
    } catch (error: any) {
      console.error('Error adding telemetry type:', error.message);
      toast.error('Failed to add telemetry type');
    } finally {
      setIsLoading(false);
    }
  };
  
  const deleteCustomTelemetryType = async (typeToDelete: string) => {
    try {
      setIsLoading(true);
      
      const updatedTypes = customTelemetryTypes.filter(type => type !== typeToDelete);
      
      const { error } = await supabase
        .from('profiles')
        .update({ custom_telemetry_types: updatedTypes })
        .eq('id', user?.id);
        
      if (error) throw error;
      
      setCustomTelemetryTypes(updatedTypes);
      toast.success('Telemetry type removed');
    } catch (error: any) {
      console.error('Error removing telemetry type:', error.message);
      toast.error('Failed to remove telemetry type');
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
      
      // Determine if this is a custom telemetry alert
      const isCustomTelemetry = !Object.keys(alertTypeMetadata).includes(alertType) && customTelemetryTypes.includes(alertType);
      
      if (existingAlertIndex >= 0) {
        // Update existing alert
        updatedAlerts = [...customAlerts];
        updatedAlerts[existingAlertIndex] = {
          ...updatedAlerts[existingAlertIndex],
          threshold: alertThreshold,
          enabled: true,
          andCondition: useAndCondition,
          additionalConditions: updatedAlerts[existingAlertIndex].additionalConditions || [],
          customTelemetry: isCustomTelemetry
        };
      } else {
        // Add new alert
        updatedAlerts = [
          ...customAlerts,
          {
            type: alertType,
            threshold: alertThreshold,
            enabled: true,
            andCondition: useAndCondition,
            additionalConditions: [],
            customTelemetry: isCustomTelemetry
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

  const addConditionToAlert = async (alertType: AlertType) => {
    try {
      setIsLoading(true);
      
      const updatedAlerts = customAlerts.map(alert => {
        if (alert.type === alertType) {
          // Create a new additional condition
          const newCondition: AlertCondition = {
            type: additionalConditionType,
            threshold: additionalThreshold
          };
          
          // Add the condition to the alert
          return {
            ...alert,
            additionalConditions: [...(alert.additionalConditions || []), newCondition]
          };
        }
        return alert;
      });
      
      const { error } = await supabase
        .from('profiles')
        .update({ custom_alerts: updatedAlerts })
        .eq('id', user?.id);
        
      if (error) throw error;
      
      setCustomAlerts(updatedAlerts);
      setIsAddingCondition(false);
      toast.success('Additional condition added');
    } catch (error: any) {
      console.error('Error adding condition:', error.message);
      toast.error('Failed to add condition');
    } finally {
      setIsLoading(false);
    }
  };

  const removeConditionFromAlert = async (alertType: AlertType, conditionIndex: number) => {
    try {
      setIsLoading(true);
      
      const updatedAlerts = customAlerts.map(alert => {
        if (alert.type === alertType && alert.additionalConditions) {
          // Remove the condition at the specified index
          const updatedConditions = [...alert.additionalConditions];
          updatedConditions.splice(conditionIndex, 1);
          
          return {
            ...alert,
            additionalConditions: updatedConditions
          };
        }
        return alert;
      });
      
      const { error } = await supabase
        .from('profiles')
        .update({ custom_alerts: updatedAlerts })
        .eq('id', user?.id);
        
      if (error) throw error;
      
      setCustomAlerts(updatedAlerts);
      toast.success('Condition removed');
    } catch (error: any) {
      console.error('Error removing condition:', error.message);
      toast.error('Failed to remove condition');
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

  const toggleAlertAndCondition = async (type: AlertType) => {
    try {
      const updatedAlerts = customAlerts.map(alert => {
        if (alert.type === type) {
          return { ...alert, andCondition: !alert.andCondition };
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
      toast.success(`AND condition ${alert?.andCondition ? 'enabled' : 'disabled'}`);
    } catch (error: any) {
      console.error('Error toggling AND condition:', error.message);
      toast.error('Failed to update alert settings');
    }
  };

  const toggleConditionsPanel = (alertType: AlertType) => {
    setIsOpenConditions(prev => ({
      ...prev,
      [alertType]: !prev[alertType]
    }));
  };

  // Helper function to get alert metadata with fallback for custom types
  const getAlertMetadata = (type: AlertType): AlertTypeMetadata => {
    if (type in alertTypeMetadata) {
      return alertTypeMetadata[type];
    }
    
    // Return default metadata for custom telemetry types
    return {
      label: type,
      description: `Alert on ${type} value`,
      icon: <AlertTriangle className="h-4 w-4 mr-2" />,
      hasThreshold: true,
      defaultThreshold: 50,
      thresholdLabel: "Value",
      thresholdMin: 0,
      thresholdMax: 999
    };
  };

  const getAlertIcon = (type: AlertType) => {
    return getAlertMetadata(type).icon;
  };

  const getAlertLabel = (type: AlertType) => {
    return getAlertMetadata(type).label;
  };

  const getAlertDescription = (alert: AlertThreshold) => {
    const metadata = getAlertMetadata(alert.type);
    const condCount = alert.additionalConditions?.length || 0;
    const condText = condCount > 0 
      ? `with ${condCount} additional condition${condCount > 1 ? 's' : ''} (${alert.andCondition ? 'AND' : 'OR'})` 
      : "";
    
    if (metadata.hasThreshold) {
      return `Alert when ${alert.type} ${metadata.description.includes("below") ? "is below" : "exceeds"} ${alert.threshold}${metadata.thresholdUnit || ''} ${condText}`;
    } else {
      return `${metadata.description} ${condText}`;
    }
  };

  const getConditionDescription = (condition: AlertCondition) => {
    const metadata = getAlertMetadata(condition.type);
    
    if (metadata.hasThreshold) {
      return `${metadata.label} ${metadata.description.includes("below") ? "below" : "exceeds"} ${condition.threshold}${metadata.thresholdUnit || ''}`;
    } else {
      return metadata.description;
    }
  };

  // Combine standard and custom telemetry types for alert type selection
  const getAllAlertTypes = () => {
    const standardTypes = Object.keys(alertTypeMetadata);
    return [...standardTypes, ...customTelemetryTypes];
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

          {/* New Card for Custom Telemetry Types */}
          <Card>
            <CardHeader>
              <CardTitle>Custom Telemetry Types</CardTitle>
              <CardDescription>
                Define custom telemetry data types for your robots
              </CardDescription>
            </CardHeader>
            <CardContent>
              <div className="space-y-4">
                <div className="flex items-end gap-2">
                  <div className="flex-1">
                    <Label htmlFor="new-telemetry-type">Add New Telemetry Type</Label>
                    <Input 
                      id="new-telemetry-type"
                      placeholder="Enter telemetry type name" 
                      value={newTelemetryType}
                      onChange={(e) => setNewTelemetryType(e.target.value)}
                      disabled={isLoading}
                    />
                  </div>
                  <Button 
                    onClick={addCustomTelemetryType}
                    disabled={isLoading || !newTelemetryType.trim()}
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
                
                {!isLoading && customTelemetryTypes.length === 0 && (
                  <div className="text-center text-muted-foreground py-4">
                    No custom telemetry types added yet
                  </div>
                )}
                
                <div className="flex flex-wrap gap-2 mt-4">
                  {customTelemetryTypes.map((type, index) => (
                    <div 
                      key={index} 
                      className="bg-secondary text-secondary-foreground px-3 py-1 rounded-full flex items-center gap-1"
                    >
                      <span>{type}</span>
                      <button 
                        onClick={() => deleteCustomTelemetryType(type)}
                        className="hover:text-destructive transition-colors"
                        disabled={isLoading}
                        aria-label={`Delete ${type} telemetry type`}
                      >
                        <X className="h-3 w-3" />
                      </button>
                    </div>
                  ))}
                </div>
              </div>
            </CardContent>
            <CardFooter className="text-sm text-muted-foreground">
              Custom telemetry types can be used for creating alerts and monitoring robot data
            </CardFooter>
          </Card>

          <Card className="md:col-span-2">
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
                        value={alertType} 
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
                          {customTelemetryTypes.length > 0 && (
                            <>
                              <SelectItem disabled>
                                <span className="text-xs text-muted-foreground">Custom Telemetry Types</span>
                              </SelectItem>
                              {customTelemetryTypes.map((type, index) => (
                                <SelectItem key={index} value={type}>{type}</SelectItem>
                              ))}
                            </>
                          )}
                        </SelectContent>
                      </Select>
                    </div>

                    <div>
                      <Label htmlFor="alert-threshold">
                        {(() => {
                          const metadata = getAlertMetadata(alertType);
                          return metadata.hasThreshold 
                            ? `${metadata.thresholdLabel || 'Threshold'}${metadata.thresholdUnit ? ' (' + metadata.thresholdUnit + ')' : ''}` 
                            : 'Threshold';
                        })()}
                      </Label>
                      <Input 
                        id="alert-threshold"
                        type="number" 
                        value={alertThreshold}
                        onChange={(e) => setAlertThreshold(Number(e.target.value))}
                        disabled={!getAlertMetadata(alertType).hasThreshold}
                        min={getAlertMetadata(alertType).thresholdMin || 0}
                        max={getAlertMetadata(alertType).thresholdMax || 100}
                      />
                    </div>
                  </div>
                  
                  <div className="flex items-center space-x-2">
                    <Switch 
                      id="and-condition" 
                      checked={useAndCondition}
                      onCheckedChange={setUseAndCondition}
                    />
                    <Label htmlFor="and-condition">
                      Combine with other conditions using AND logic (default is OR)
                    </Label>
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
                      <div key={index} className={`bg-muted p-3 rounded-md ${alert.customTelemetry ? 'border-l-4 border-purple-500' : ''}`}>
                        <div className="flex items-center justify-between">
                          <div className="flex items-center">
                            {getAlertIcon(alert.type)}
                            <div>
                              <p className="font-medium">{getAlertLabel(alert.type)}</p>
                              <p className="text-sm text-muted-foreground">{getAlertDescription(alert)}</p>
                              {alert.customTelemetry && (
                                <span className="inline-flex items-center px-2 py-0.5 rounded text-xs font-medium bg-purple-100 text-purple-800 dark:bg-purple-900/30 dark:text-purple-300 mt-1">
                                  Custom Telemetry
                                </span>
                              )}
                            </div>
                          </div>
                          <div className="flex items-center gap-2">
                            <Button 
                              variant="outline" 
                              size="sm"
                              onClick={() => toggleAlertAndCondition(alert.type)}
                              className={alert.enabled ? "" : "opacity-50"}
                              disabled={!alert.enabled}
                            >
                              {alert.andCondition ? "AND" : "OR"}
                            </Button>
                            <Button 
                              variant={alert.enabled ? "default" : "outline"} 
                              size="sm"
                              onClick={() => toggleAlertEnabled(alert.type)}
                            >
                              {alert.enabled ? "Enabled" : "Disabled"}
                            </Button>
                          </div>
                        </div>
                        
                        {/* Additional Conditions Panel */}
                        <Collapsible 
                          open={isOpenConditions[alert.type]} 
                          onOpenChange={() => toggleConditionsPanel(alert.type)}
                          className="mt-2"
                        >
                          <CollapsibleTrigger asChild>
                            <Button variant="ghost" size="sm" className="w-full flex justify-between p-2 h-8">
                              <span>Additional Conditions</span>
                              <span>{isOpenConditions[alert.type] ? "Hide" : "Show"}</span>
                            </Button>
                          </CollapsibleTrigger>
                          <CollapsibleContent className="space-y-2 pt-2">
                            {/* List existing conditions */}
                            {alert.additionalConditions && alert.additionalConditions.length > 0 ? (
                              <div className="space-y-2">
                                {alert.additionalConditions.map((condition, condIndex) => (
                                  <div key={condIndex} className="flex items-center justify-between bg-background p-2 rounded border">
                                    <span>{getConditionDescription(condition)}</span>
                                    <Button 
                                      variant="ghost" 
                                      size="sm"
                                      onClick={() => removeConditionFromAlert(alert.type, condIndex)}
                                    >
                                      <X className="h-4 w-4" />
                                    </Button>
                                  </div>
                                ))}
                              </div>
                            ) : (
                              <p className="text-sm text-muted-foreground">No additional conditions</p>
                            )}
                            
                            {/* Add new condition UI */}
                            {selectedAlert?.type === alert.type && isAddingCondition ? (
                              <div className="bg-background p-3 rounded border space-y-2">
                                <div className="grid grid-cols-2 gap-2">
                                  <div>
                                    <Label htmlFor="condition-type">Condition Type</Label>
                                    <Select 
                                      value={additionalConditionType} 
                                      onValueChange={(value) => setAdditionalConditionType(value as AlertType)}
                                    >
                                      <SelectTrigger id="condition-type">
                                        <SelectValue placeholder="Select condition" />
                                      </SelectTrigger>
                                      <SelectContent>
                                        <SelectItem value="battery">Low Battery</SelectItem>
                                        <SelectItem value="temperature">High Temperature</SelectItem>
                                        <SelectItem value="offline">Offline Status</SelectItem>
                                        <SelectItem value="error">Errors</SelectItem>
                                        {customTelemetryTypes.length > 0 && (
                                          <>
                                            <SelectItem disabled>
                                              <span className="text-xs text-muted-foreground">Custom Telemetry Types</span>
                                            </SelectItem>
                                            {customTelemetryTypes.map((type, index) => (
                                              <SelectItem key={index} value={type}>{type}</SelectItem>
                                            ))}
                                          </>
                                        )}
                                      </SelectContent>
                                    </Select>
                                  </div>
                                  <div>
                                    <Label htmlFor="condition-threshold">Threshold</Label>
                                    <Input 
                                      id="condition-threshold"
                                      type="number" 
                                      value={additionalThreshold}
                                      onChange={(e) => setAdditionalThreshold(Number(e.target.value))}
                                      disabled={!getAlertMetadata(additionalConditionType).hasThreshold}
                                    />
                                  </div>
                                </div>
                                <div className="flex justify-end gap-2">
                                  <Button 
                                    variant="outline" 
                                    size="sm" 
                                    onClick={() => setIsAddingCondition(false)}
                                  >
                                    Cancel
                                  </Button>
                                  <Button 
                                    size="sm" 
                                    onClick={() => addConditionToAlert(alert.type)}
                                  >
                                    Add
                                  </Button>
                                </div>
                              </div>
                            ) : (
                              <Button 
                                variant="outline" 
                                size="sm" 
                                className="w-full mt-1"
                                onClick={() => {
                                  setSelectedAlert(alert);
                                  setIsAddingCondition(true);
                                }}
                              >
                                <PlusCircle className="h-4 w-4 mr-2" />
                                Add Condition
                              </Button>
                            )}
                          </CollapsibleContent>
                        </Collapsible>
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
