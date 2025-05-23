
import { useState, useEffect } from "react";
import { useRobots } from "@/hooks/useRobots";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { Textarea } from "@/components/ui/textarea";
import { Plus, Loader2 } from "lucide-react";
import { useToast } from "@/hooks/use-toast";
import { supabase } from "@/integrations/supabase/client";
import { useAuth } from "@/context/AuthContext";
import { useSubscriptionLimits } from "@/utils/planRestrictions";

export function AddRobotModal({ disabled = false }: { disabled?: boolean }) {
  const [open, setOpen] = useState(false);
  const [name, setName] = useState("");
  const [type, setType] = useState("");
  const [description, setDescription] = useState("");
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [customRobotTypes, setCustomRobotTypes] = useState<string[]>([]);
  const { fetchRobots } = useRobots(); // Changed from addRobot to fetchRobots
  const { toast } = useToast();
  const { user, session } = useAuth(); // Added session to get auth token

  // Default robot types
  const defaultRobotTypes = [
    "delivery",
    "warehouse",
    "cleaning",
    "security",
    "assembly"
  ];

  // Fetch custom robot types when modal opens
  useEffect(() => {
    if (open && user) {
      fetchCustomRobotTypes();
    }
  }, [open, user]);

  // Fetch user's custom robot types
  const fetchCustomRobotTypes = async () => {
    try {
      const { data, error } = await supabase
        .from('profiles')
        .select('custom_robot_types')
        .eq('id', user?.id)
        .single();
        
      if (error) {
        console.error("Error fetching custom robot types:", error);
        return;
      }
      
      if (data?.custom_robot_types) {
        setCustomRobotTypes(data.custom_robot_types);
      }
    } catch (error) {
      console.error("Error in fetchCustomRobotTypes:", error);
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    if (!name || !type) {
      toast({
        title: "Missing fields",
        description: "Please fill in all required fields",
        variant: "destructive"
      });
      return;
    }
    
    if (!session?.access_token) {
      toast({
        title: "Authentication error",
        description: "Please log in to add a robot",
        variant: "destructive"
      });
      return;
    }
    
    setIsSubmitting(true);

    try {
      // Get the base URL from the client's configuration
      const supabaseUrl = new URL(supabase.getUrl()).origin;
      
      // Use the edge function to create robot with API key
      const response = await fetch(`${supabaseUrl}/functions/v1/create-robot-with-api-key`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${session.access_token}`
        },
        body: JSON.stringify({
          name,
          type,
          description: description || null
        })
      });
      
      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.error || "Failed to create robot");
      }
      
      const result = await response.json();
      
      // Reset form and close dialog
      setName("");
      setType("");
      setDescription("");
      setOpen(false);
      
      // Refresh the robots list
      fetchRobots();
      
      toast({
        title: "Robot added",
        description: `${name} has been successfully added.`
      });
    } catch (error: any) {
      console.error("Error adding robot:", error);
      toast({
        title: "Error",
        description: error.message || "Failed to add robot. Please try again.",
        variant: "destructive"
      });
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <Dialog open={open} onOpenChange={setOpen}>
      <DialogTrigger asChild>
        <Button className="z-10" disabled={disabled}>
          <Plus className="mr-2 h-4 w-4" />
          Add Robot
        </Button>
      </DialogTrigger>
      <DialogContent className="sm:max-w-[425px]">
        <form onSubmit={handleSubmit}>
          <DialogHeader>
            <DialogTitle>Add New Robot</DialogTitle>
            <DialogDescription>
              Register a new robot to monitor its status and telemetry.
            </DialogDescription>
          </DialogHeader>
          <div className="grid gap-4 py-4">
            <div className="grid grid-cols-4 items-center gap-4">
              <Label htmlFor="name" className="text-right">
                Name
              </Label>
              <Input
                id="name"
                value={name}
                onChange={(e) => setName(e.target.value)}
                className="col-span-3"
                required
              />
            </div>
            <div className="grid grid-cols-4 items-center gap-4">
              <Label htmlFor="type" className="text-right">
                Type
              </Label>
              <div className="col-span-3 w-full">
                <Select
                  value={type}
                  onValueChange={setType}
                  required
                >
                  <SelectTrigger id="type">
                    <SelectValue placeholder="Select robot type" />
                  </SelectTrigger>
                  <SelectContent>
                    {/* Default robot types */}
                    <SelectItem value="delivery">Delivery Bot</SelectItem>
                    <SelectItem value="warehouse">Warehouse Bot</SelectItem>
                    <SelectItem value="cleaning">Cleaning Bot</SelectItem>
                    <SelectItem value="security">Security Bot</SelectItem>
                    <SelectItem value="assembly">Assembly Bot</SelectItem>
                    
                    {/* Custom robot types */}
                    {customRobotTypes.length > 0 && (
                      <>
                        <div className="px-2 py-1.5 text-xs font-medium text-muted-foreground">
                          Custom Types
                        </div>
                        {customRobotTypes.map((customType) => (
                          <SelectItem key={customType} value={customType}>
                            {customType}
                          </SelectItem>
                        ))}
                      </>
                    )}
                  </SelectContent>
                </Select>
              </div>
            </div>
            <div className="grid grid-cols-4 items-center gap-4">
              <Label htmlFor="description" className="text-right">
                Description
              </Label>
              <Textarea
                id="description"
                value={description}
                onChange={(e) => setDescription(e.target.value)}
                className="col-span-3"
                placeholder="Optional details about the robot"
              />
            </div>
          </div>
          <DialogFooter>
            <Button type="submit" disabled={isSubmitting}>
              {isSubmitting ? (
                <>
                  <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                  Adding...
                </>
              ) : (
                "Add Robot"
              )}
            </Button>
          </DialogFooter>
        </form>
      </DialogContent>
    </Dialog>
  );
}
