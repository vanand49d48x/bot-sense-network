
import { useState } from "react";
import { useToast } from "@/hooks/use-toast";
import { Button } from "@/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardFooter,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import { supabase } from "@/integrations/supabase/client";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { RefreshCw, Loader2 } from "lucide-react";
import {
  AlertDialog,
  AlertDialogAction,
  AlertDialogCancel,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
} from "@/components/ui/alert-dialog";

export default function AdminSettings() {
  const { toast } = useToast();
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [newAdminEmail, setNewAdminEmail] = useState<string>("");
  const [showConfirmDialog, setShowConfirmDialog] = useState<boolean>(false);

  const handleCreateFirstAdmin = async () => {
    setIsLoading(true);
    try {
      const { error } = await supabase.rpc('create_first_admin', {
        admin_email: newAdminEmail
      });

      if (error) throw error;

      toast({
        title: "Admin created successfully",
        description: `${newAdminEmail} has been granted admin privileges.`,
      });
      setNewAdminEmail("");
    } catch (error: any) {
      console.error("Error creating admin:", error);
      toast({
        title: "Error creating admin",
        description: error.message || "Could not create admin user",
        variant: "destructive",
      });
    } finally {
      setIsLoading(false);
      setShowConfirmDialog(false);
    }
  };

  return (
    <>
      <div className="space-y-6">
        <div>
          <h1 className="text-3xl font-bold tracking-tight">System Settings</h1>
          <p className="text-muted-foreground">
            Manage system-wide settings and configurations
          </p>
        </div>

        <Card>
          <CardHeader>
            <CardTitle>Admin Access Management</CardTitle>
            <CardDescription>
              Create, manage, and revoke administrator access
            </CardDescription>
          </CardHeader>
          <CardContent className="space-y-4">
            <div className="space-y-2">
              <Label htmlFor="new-admin">Grant Admin Access</Label>
              <div className="flex gap-2">
                <Input 
                  id="new-admin"
                  placeholder="user@example.com"
                  value={newAdminEmail}
                  onChange={(e) => setNewAdminEmail(e.target.value)}
                />
                <Button
                  onClick={() => setShowConfirmDialog(true)}
                  disabled={isLoading || !newAdminEmail || !newAdminEmail.includes('@')}
                >
                  {isLoading ? (
                    <>
                      <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                      Processing
                    </>
                  ) : (
                    "Grant Access"
                  )}
                </Button>
              </div>
              <p className="text-sm text-muted-foreground">
                Enter the email address of the user you want to grant administrator privileges to
              </p>
            </div>
          </CardContent>
        </Card>

        <Card>
          <CardHeader>
            <CardTitle>System Maintenance</CardTitle>
            <CardDescription>
              Perform system maintenance tasks
            </CardDescription>
          </CardHeader>
          <CardContent className="space-y-4">
            <div>
              <Button variant="outline" className="flex items-center gap-2">
                <RefreshCw className="h-4 w-4" />
                Refresh System Cache
              </Button>
            </div>
          </CardContent>
          <CardFooter>
            <p className="text-sm text-muted-foreground">
              Note: System maintenance operations may temporarily affect performance
            </p>
          </CardFooter>
        </Card>
      </div>

      <AlertDialog open={showConfirmDialog} onOpenChange={setShowConfirmDialog}>
        <AlertDialogContent>
          <AlertDialogHeader>
            <AlertDialogTitle>Grant Admin Access</AlertDialogTitle>
            <AlertDialogDescription>
              Are you sure you want to grant administrator privileges to <strong>{newAdminEmail}</strong>?
              <br /><br />
              This will give them full access to all system features and data.
            </AlertDialogDescription>
          </AlertDialogHeader>
          <AlertDialogFooter>
            <AlertDialogCancel>Cancel</AlertDialogCancel>
            <AlertDialogAction onClick={handleCreateFirstAdmin}>
              Grant Admin Access
            </AlertDialogAction>
          </AlertDialogFooter>
        </AlertDialogContent>
      </AlertDialog>
    </>
  );
}
