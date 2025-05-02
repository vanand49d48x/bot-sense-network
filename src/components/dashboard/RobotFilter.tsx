
import { useState, useEffect } from "react";
import { Robot, UserProfile } from "@/types/robot";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { Slider } from "@/components/ui/slider";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { X } from "lucide-react";

interface RobotFilterProps {
  robots: Robot[];
  userProfile: UserProfile | null;
  onFilteredRobotsChange: (robots: Robot[]) => void;
}

export function RobotFilter({ robots, userProfile, onFilteredRobotsChange }: RobotFilterProps) {
  const [nameFilter, setNameFilter] = useState("");
  const [modelFilter, setModelFilter] = useState<string | null>(null);
  const [statusFilter, setStatusFilter] = useState<string | null>(null);
  const [batteryFilter, setBatteryFilter] = useState([0, 100]);
  const [customFilters, setCustomFilters] = useState<Array<{type: string, value: string, operator: string}>>([]);
  
  // Get available models from robots
  const availableModels = Array.from(new Set(robots.map(r => r.model)));
  
  // Get available custom telemetry types that exist in the robots' data
  const availableCustomTypes = new Set<string>();
  robots.forEach(robot => {
    if (robot.telemetryData) {
      Object.keys(robot.telemetryData).forEach(key => {
        availableCustomTypes.add(key);
      });
    }
  });
  
  // See if user has defined custom types
  const userDefinedTypes = userProfile?.custom_telemetry_types || [];
  
  // Combine both sources
  const allCustomTypes = Array.from(new Set([...Array.from(availableCustomTypes), ...userDefinedTypes]));
  
  // Apply filters when they change
  useEffect(() => {
    let filtered = [...robots];
    
    // Apply name filter
    if (nameFilter) {
      filtered = filtered.filter(robot => 
        robot.name.toLowerCase().includes(nameFilter.toLowerCase())
      );
    }
    
    // Apply model filter
    if (modelFilter) {
      filtered = filtered.filter(robot => robot.model === modelFilter);
    }
    
    // Apply status filter
    if (statusFilter) {
      filtered = filtered.filter(robot => robot.status === statusFilter);
    }
    
    // Apply battery filter
    filtered = filtered.filter(robot => 
      robot.batteryLevel >= batteryFilter[0] && 
      robot.batteryLevel <= batteryFilter[1]
    );
    
    // Apply custom filters
    customFilters.forEach(filter => {
      filtered = filtered.filter(robot => {
        if (!robot.telemetryData || !(filter.type in robot.telemetryData)) {
          return false;
        }
        
        const value = robot.telemetryData[filter.type];
        const filterValue = filter.value;
        
        // Try to convert to numbers for numeric comparisons
        const numValue = !isNaN(Number(value)) ? Number(value) : null;
        const numFilterValue = !isNaN(Number(filterValue)) ? Number(filterValue) : null;
        
        if (numValue !== null && numFilterValue !== null) {
          switch (filter.operator) {
            case 'eq': return numValue === numFilterValue;
            case 'gt': return numValue > numFilterValue;
            case 'lt': return numValue < numFilterValue;
            case 'gte': return numValue >= numFilterValue;
            case 'lte': return numValue <= numFilterValue;
          }
        } else {
          // String comparison
          const strValue = String(value).toLowerCase();
          const strFilterValue = filterValue.toLowerCase();
          
          switch (filter.operator) {
            case 'eq': return strValue === strFilterValue;
            case 'contains': return strValue.includes(strFilterValue);
            default: return strValue === strFilterValue;
          }
        }
      });
    });
    
    onFilteredRobotsChange(filtered);
  }, [nameFilter, modelFilter, statusFilter, batteryFilter, customFilters, robots, onFilteredRobotsChange]);
  
  const addCustomFilter = () => {
    if (allCustomTypes.length === 0) return;
    
    setCustomFilters([
      ...customFilters, 
      {
        type: allCustomTypes[0],
        value: "",
        operator: "eq"
      }
    ]);
  };
  
  const removeCustomFilter = (index: number) => {
    setCustomFilters(customFilters.filter((_, i) => i !== index));
  };
  
  const updateCustomFilter = (index: number, field: string, value: string) => {
    const updated = [...customFilters];
    updated[index] = { ...updated[index], [field]: value };
    setCustomFilters(updated);
  };
  
  const clearFilters = () => {
    setNameFilter("");
    setModelFilter(null);
    setStatusFilter(null);
    setBatteryFilter([0, 100]);
    setCustomFilters([]);
  };
  
  return (
    <div className="space-y-4">
      <div className="flex justify-between items-center">
        <h2 className="text-xl font-semibold">Filter Robots</h2>
        <Button variant="outline" size="sm" onClick={clearFilters}>Clear All</Button>
      </div>
      
      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        {/* Name filter */}
        <div>
          <Label htmlFor="name-filter">Robot Name</Label>
          <Input 
            id="name-filter"
            placeholder="Filter by name..."
            value={nameFilter}
            onChange={(e) => setNameFilter(e.target.value)}
          />
        </div>
        
        {/* Model filter */}
        <div>
          <Label htmlFor="model-filter">Model</Label>
          <Select 
            value={modelFilter || ""} 
            onValueChange={(value) => setModelFilter(value || null)}
          >
            <SelectTrigger id="model-filter">
              <SelectValue placeholder="All models" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="">All models</SelectItem>
              {availableModels.map((model) => (
                <SelectItem key={model} value={model}>{model}</SelectItem>
              ))}
            </SelectContent>
          </Select>
        </div>
        
        {/* Status filter */}
        <div>
          <Label htmlFor="status-filter">Status</Label>
          <Select 
            value={statusFilter || ""} 
            onValueChange={(value) => setStatusFilter(value || null)}
          >
            <SelectTrigger id="status-filter">
              <SelectValue placeholder="Any status" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="">Any status</SelectItem>
              <SelectItem value="online">Online</SelectItem>
              <SelectItem value="offline">Offline</SelectItem>
              <SelectItem value="warning">Warning</SelectItem>
            </SelectContent>
          </Select>
        </div>
      </div>
      
      {/* Battery level filter */}
      <div className="space-y-2">
        <div className="flex items-center justify-between">
          <Label>Battery Level: {batteryFilter[0]}% - {batteryFilter[1]}%</Label>
        </div>
        <Slider
          defaultValue={[0, 100]}
          max={100}
          step={1}
          value={batteryFilter}
          onValueChange={(value) => setBatteryFilter(value)}
          className="py-4"
        />
      </div>
      
      {/* Active filters display */}
      <div className="flex flex-wrap gap-2 mt-2">
        {nameFilter && (
          <Badge variant="secondary" className="flex items-center gap-1">
            Name: {nameFilter}
            <X 
              className="h-3 w-3 cursor-pointer" 
              onClick={() => setNameFilter("")}
            />
          </Badge>
        )}
        
        {modelFilter && (
          <Badge variant="secondary" className="flex items-center gap-1">
            Model: {modelFilter}
            <X 
              className="h-3 w-3 cursor-pointer" 
              onClick={() => setModelFilter(null)}
            />
          </Badge>
        )}
        
        {statusFilter && (
          <Badge variant="secondary" className="flex items-center gap-1">
            Status: {statusFilter}
            <X 
              className="h-3 w-3 cursor-pointer" 
              onClick={() => setStatusFilter(null)}
            />
          </Badge>
        )}
        
        {(batteryFilter[0] > 0 || batteryFilter[1] < 100) && (
          <Badge variant="secondary" className="flex items-center gap-1">
            Battery: {batteryFilter[0]}% - {batteryFilter[1]}%
            <X 
              className="h-3 w-3 cursor-pointer" 
              onClick={() => setBatteryFilter([0, 100])}
            />
          </Badge>
        )}
      </div>
      
      {/* Custom telemetry filters */}
      {allCustomTypes.length > 0 && (
        <div className="space-y-4">
          <div className="flex justify-between items-center">
            <h3 className="font-medium">Custom Telemetry Filters</h3>
            <Button variant="outline" size="sm" onClick={addCustomFilter}>Add Filter</Button>
          </div>
          
          {customFilters.map((filter, index) => (
            <div key={index} className="grid grid-cols-3 gap-2 items-end bg-muted p-2 rounded-md">
              <div>
                <Label>Data Type</Label>
                <Select 
                  value={filter.type} 
                  onValueChange={(value) => updateCustomFilter(index, "type", value)}
                >
                  <SelectTrigger>
                    <SelectValue />
                  </SelectTrigger>
                  <SelectContent>
                    {allCustomTypes.map((type) => (
                      <SelectItem key={type} value={type}>{type}</SelectItem>
                    ))}
                  </SelectContent>
                </Select>
              </div>
              <div>
                <Label>Operator</Label>
                <Select 
                  value={filter.operator} 
                  onValueChange={(value) => updateCustomFilter(index, "operator", value)}
                >
                  <SelectTrigger>
                    <SelectValue />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="eq">Equals</SelectItem>
                    <SelectItem value="contains">Contains</SelectItem>
                    <SelectItem value="gt">Greater Than</SelectItem>
                    <SelectItem value="lt">Less Than</SelectItem>
                    <SelectItem value="gte">Greater or Equal</SelectItem>
                    <SelectItem value="lte">Less or Equal</SelectItem>
                  </SelectContent>
                </Select>
              </div>
              <div className="flex gap-2">
                <div className="flex-1">
                  <Label>Value</Label>
                  <Input 
                    value={filter.value}
                    onChange={(e) => updateCustomFilter(index, "value", e.target.value)}
                  />
                </div>
                <Button 
                  variant="ghost" 
                  size="icon" 
                  onClick={() => removeCustomFilter(index)}
                  className="self-end"
                >
                  <X className="h-4 w-4" />
                </Button>
              </div>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}
