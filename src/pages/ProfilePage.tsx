
import { useState, useEffect } from "react";
import MainLayout from "@/components/layout/MainLayout";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { useAuth } from "@/context/AuthContext";
import { toast } from "@/components/ui/sonner";
import { PlusCircle, Loader2, X } from "lucide-react";
import { supabase } from "@/integrations/supabase/client";

export default function ProfilePage() {
  const { user } = useAuth();
  const [customRobotTypes, setCustomRobotTypes] = useState<string[]>([]);
  const [newType, setNewType] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  
  useEffect(() => {
    if (user) {
      fetchCustomRobotTypes();
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

  return (
    <MainLayout>
      <div className="container py-6">
        <h1 className="text-3xl font-bold mb-6">Profile Settings</h1>
        
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
      </div>
    </MainLayout>
  );
}
