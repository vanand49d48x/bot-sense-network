import { Robot } from "@/types/robot";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { LeafletMap } from "./LeafletMap";
import { useState } from "react";
import { Button } from "@/components/ui/button";
import { Check, ChevronDown } from "lucide-react";
import {
  DropdownMenu,
  DropdownMenuCheckboxItem,
  DropdownMenuContent,
  DropdownMenuTrigger
} from "@/components/ui/dropdown-menu";

interface MapViewProps {
  robots: Robot[];
  selectedRobotIds?: string[];
  onSelectRobots?: (robotIds: string[]) => void;
}

export function MapView({ robots, selectedRobotIds = ['all'], onSelectRobots }: MapViewProps) {
  // State to track if dropdown is open
  const [dropdownOpen, setDropdownOpen] = useState(false);
  // Local state to track selections
  const [selectedIds, setSelectedIds] = useState<string[]>(selectedRobotIds);
  
  // Filter robots that have location data
  const robotsWithLocation = robots.filter(robot => robot.location !== undefined);
  
  // Apply filter by selected robot ids if provided
  const filteredRobots = selectedIds && selectedIds.length > 0 && !selectedIds.includes('all')
    ? robotsWithLocation.filter(robot => selectedIds.includes(robot.id))
    : robotsWithLocation;

  // Handle robot selection
  const handleRobotSelection = (robotId: string) => {
    let newSelection: string[];
    
    // If "all" is being selected, return just ["all"]
    if (robotId === 'all') {
      newSelection = ['all'];
    } else {
      // If an individual robot is selected, we need to remove "all" from the selection
      const withoutAll = selectedIds.filter(id => id !== 'all');
      
      // If the robot is already selected, remove it
      if (withoutAll.includes(robotId)) {
        newSelection = withoutAll.filter(id => id !== robotId);
        // If no robots are selected, default back to "all"
        if (newSelection.length === 0) {
          newSelection = ['all'];
        }
      } 
      // Otherwise add the robot to the selection
      else {
        newSelection = [...withoutAll, robotId];
      }
    }
    
    setSelectedIds(newSelection);
    if (onSelectRobots) {
      onSelectRobots(newSelection);
    }
    
    // Importantly, we're not closing the dropdown here
  };

  if (filteredRobots.length === 0) {
    return null;
  }

  return (
    <Card className="mt-6 animate-fade-in">
      <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
        <CardTitle>Robot Locations</CardTitle>
        <div className="relative">
          <DropdownMenu open={dropdownOpen} onOpenChange={setDropdownOpen}>
            <DropdownMenuTrigger asChild>
              <Button variant="outline" size="sm" className="ml-auto">
                {selectedIds.includes('all') 
                  ? 'All Robots' 
                  : `${selectedIds.length} robot${selectedIds.length > 1 ? 's' : ''} selected`}
                <ChevronDown className="ml-2 h-4 w-4" />
              </Button>
            </DropdownMenuTrigger>
            <DropdownMenuContent align="end" className="w-[200px]">
              <DropdownMenuCheckboxItem
                checked={selectedIds.includes('all')}
                onSelect={(e) => {
                  e.preventDefault();
                  handleRobotSelection('all');
                }}
              >
                All Robots
              </DropdownMenuCheckboxItem>
              {robots.map(robot => (
                <DropdownMenuCheckboxItem
                  key={robot.id}
                  checked={selectedIds.includes(robot.id)}
                  onSelect={(e) => {
                    e.preventDefault();
                    handleRobotSelection(robot.id);
                  }}
                >
                  {robot.name}
                </DropdownMenuCheckboxItem>
              ))}
            </DropdownMenuContent>
          </DropdownMenu>
        </div>
      </CardHeader>
      <CardContent>
        <div className="h-64 rounded-md overflow-hidden">
          <LeafletMap robots={filteredRobots} height="100%" showTooltips={true} />
        </div>
        <div className="mt-4 text-xs text-muted-foreground">
          Showing {filteredRobots.length} robots with location data
        </div>
      </CardContent>
    </Card>
  );
}
