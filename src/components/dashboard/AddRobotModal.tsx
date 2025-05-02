
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

export function AddRobotModal() {
  const [open, setOpen] = useState(false);
  const [name, setName] = useState("");
  const [type, setType] = useState("");
  const [description, setDescription] = useState("");
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [customRobotTypes, setCustomRobotTypes] = useState<string[]>([]);
  const { addRobot } = useRobots();
  const { toast } = useToast();
  const { user } = useAuth();

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
    
    setIsSubmitting(true);

    try {
      await addRobot({
        name,
        type,
        description,
      });

      // Reset form and close dialog
      setName("");
      setType("");
      setDescription("");
      setOpen(false);
      
      toast({
        title: "Robot added",
        description: `${name} has been successfully added.`
      });
    } catch (error) {
      toast({
        title: "Error",
        description: "Failed to add robot. Please try again.",
        variant: "destructive"
      });
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <Dialog open={open} onOpenChange={setOpen}>
      <DialogTrigger asChild>
        <Button className="z-10">
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
                    {/* Fixed: Removed the placeholder SelectItem with empty value */}
                    
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
