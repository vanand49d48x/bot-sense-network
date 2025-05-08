
import { useState, useEffect } from "react";
import { useToast } from "@/hooks/use-toast";
import { supabase } from "@/integrations/supabase/client";
import { Loader2, Search, Shield, ShieldOff } from "lucide-react";
import { Input } from "@/components/ui/input";
import { Button } from "@/components/ui/button";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import { useAuth } from "@/context/AuthContext";
import { Badge } from "@/components/ui/badge";
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

interface User {
  id: string;
  email: string;
  created_at: string;
  last_sign_in_at: string | null;
  is_admin?: boolean;
}

export default function UserManagement() {
  const { toast } = useToast();
  const { user: currentUser } = useAuth();
  const [users, setUsers] = useState<User[]>([]);
  const [admins, setAdmins] = useState<string[]>([]);
  const [loading, setLoading] = useState(true);
  const [searchQuery, setSearchQuery] = useState("");
  const [isChangingAdminStatus, setIsChangingAdminStatus] = useState(false);
  const [userToChange, setUserToChange] = useState<User | null>(null);
  const [adminAction, setAdminAction] = useState<"grant" | "revoke" | null>(null);

  useEffect(() => {
    fetchUsers();
    fetchAdmins();
  }, []);

  const fetchUsers = async () => {
    setLoading(true);
    try {
      // Get all profiles (which correspond to users)
      const { data, error } = await supabase
        .from("profiles")
        .select("id, first_name, last_name, created_at");

      if (error) throw error;

      // Get more user details from auth.users - but we can't directly query this
      // So we'll get basic info from profiles and simulate it for now
      const usersWithDetails = data.map((profile: any) => ({
        id: profile.id,
        email: `${profile.first_name || ''} ${profile.last_name || ''}`.trim() || 'User',
        created_at: profile.created_at,
        last_sign_in_at: null
      }));

      setUsers(usersWithDetails);
    } catch (error: any) {
      console.error("Error fetching users:", error);
      toast({
        title: "Failed to load users",
        description: error.message || "Could not retrieve user data",
        variant: "destructive",
      });
    } finally {
      setLoading(false);
    }
  };

  const fetchAdmins = async () => {
    try {
      const { data, error } = await supabase
        .from("admin_users")
        .select("id");

      if (error) throw error;
      
      setAdmins(data.map(admin => admin.id));
    } catch (error: any) {
      console.error("Error fetching admin users:", error);
      toast({
        title: "Failed to load admin users",
        description: error.message || "Could not retrieve admin status",
        variant: "destructive",
      });
    }
  };

  const handleUpdateAdminStatus = async (userId: string, grantAdmin: boolean) => {
    setIsChangingAdminStatus(true);
    try {
      if (grantAdmin) {
        // Grant admin access
        const { error } = await supabase
          .from("admin_users")
          .insert({ 
            id: userId, 
            granted_by: currentUser?.email || currentUser?.id 
          });

        if (error) throw error;

        toast({
          title: "Admin access granted",
          description: "User now has administrator privileges",
        });
        
        setAdmins([...admins, userId]);
      } else {
        // Revoke admin access
        const { error } = await supabase
          .from("admin_users")
          .delete()
          .eq("id", userId);

        if (error) throw error;

        toast({
          title: "Admin access revoked",
          description: "User's administrator privileges have been removed",
        });
        
        setAdmins(admins.filter(id => id !== userId));
      }
    } catch (error: any) {
      console.error("Error updating admin status:", error);
      toast({
        title: "Failed to update admin status",
        description: error.message || "Could not update user permissions",
        variant: "destructive",
      });
    } finally {
      setIsChangingAdminStatus(false);
      setUserToChange(null);
      setAdminAction(null);
    }
  };

  const confirmAdminChange = (user: User, action: "grant" | "revoke") => {
    setUserToChange(user);
    setAdminAction(action);
  };

  const filteredUsers = users.filter(user =>
    user.email?.toLowerCase().includes(searchQuery.toLowerCase())
  );

  const formatDate = (dateString: string | null) => {
    if (!dateString) return "Never";
    return new Date(dateString).toLocaleDateString() + " " + 
           new Date(dateString).toLocaleTimeString();
  };

  return (
    <>
      <div className="space-y-6">
        <div>
          <h1 className="text-3xl font-bold tracking-tight">User Management</h1>
          <p className="text-muted-foreground">
            View and manage all system users
          </p>
        </div>

        <Card>
          <CardHeader>
            <CardTitle>Users</CardTitle>
            <CardDescription>
              Manage user accounts and administrative privileges
            </CardDescription>
          </CardHeader>
          <CardContent>
            <div className="mb-4 relative">
              <Search className="absolute left-2 top-2.5 h-4 w-4 text-muted-foreground" />
              <Input 
                placeholder="Search users by email..."
                className="pl-8"
                value={searchQuery}
                onChange={(e) => setSearchQuery(e.target.value)}
              />
            </div>

            {loading ? (
              <div className="flex items-center justify-center py-8">
                <Loader2 className="h-8 w-8 animate-spin text-primary" />
                <span className="ml-2">Loading users...</span>
              </div>
            ) : (
              <div className="rounded-md border">
                <Table>
                  <TableHeader>
                    <TableRow>
                      <TableHead>User Info</TableHead>
                      <TableHead>Created</TableHead>
                      <TableHead>Status</TableHead>
                      <TableHead className="text-right">Actions</TableHead>
                    </TableRow>
                  </TableHeader>
                  <TableBody>
                    {filteredUsers.length > 0 ? (
                      filteredUsers.map((user) => {
                        const isAdmin = admins.includes(user.id);
                        return (
                          <TableRow key={user.id}>
                            <TableCell>{user.email}</TableCell>
                            <TableCell>{formatDate(user.created_at)}</TableCell>
                            <TableCell>
                              {isAdmin ? (
                                <Badge variant="default" className="bg-purple-500">Admin</Badge>
                              ) : (
                                <Badge variant="outline">User</Badge>
                              )}
                            </TableCell>
                            <TableCell className="text-right">
                              {user.id === currentUser?.id ? (
                                <Button variant="outline" size="sm" disabled>
                                  Current User
                                </Button>
                              ) : isAdmin ? (
                                <Button 
                                  variant="outline" 
                                  size="sm"
                                  onClick={() => confirmAdminChange(user, "revoke")}
                                  className="flex items-center"
                                >
                                  <ShieldOff className="h-4 w-4 mr-1" /> 
                                  Revoke Admin
                                </Button>
                              ) : (
                                <Button 
                                  variant="outline" 
                                  size="sm"
                                  onClick={() => confirmAdminChange(user, "grant")}
                                  className="flex items-center"
                                >
                                  <Shield className="h-4 w-4 mr-1" /> 
                                  Make Admin
                                </Button>
                              )}
                            </TableCell>
                          </TableRow>
                        );
                      })
                    ) : (
                      <TableRow>
                        <TableCell colSpan={4} className="text-center py-4">
                          No users found
                        </TableCell>
                      </TableRow>
                    )}
                  </TableBody>
                </Table>
              </div>
            )}
          </CardContent>
        </Card>
      </div>

      <AlertDialog 
        open={!!userToChange && !!adminAction} 
        onOpenChange={() => {
          setUserToChange(null);
          setAdminAction(null);
        }}
      >
        <AlertDialogContent>
          <AlertDialogHeader>
            <AlertDialogTitle>
              {adminAction === "grant" 
                ? "Grant Administrator Access" 
                : "Revoke Administrator Access"}
            </AlertDialogTitle>
            <AlertDialogDescription>
              {adminAction === "grant"
                ? `Are you sure you want to grant administrator privileges to ${userToChange?.email}? This will give them full access to all system features and data.`
                : `Are you sure you want to revoke administrator privileges from ${userToChange?.email}? They will no longer have access to admin features.`}
            </AlertDialogDescription>
          </AlertDialogHeader>
          <AlertDialogFooter>
            <AlertDialogCancel disabled={isChangingAdminStatus}>Cancel</AlertDialogCancel>
            <AlertDialogAction
              onClick={() => userToChange && handleUpdateAdminStatus(userToChange.id, adminAction === "grant")}
              disabled={isChangingAdminStatus}
              className={adminAction === "revoke" ? "bg-destructive hover:bg-destructive/90" : ""}
            >
              {isChangingAdminStatus ? (
                <>
                  <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                  Processing...
                </>
              ) : adminAction === "grant" ? (
                "Grant Admin Access"
              ) : (
                "Revoke Admin Access"
              )}
            </AlertDialogAction>
          </AlertDialogFooter>
        </AlertDialogContent>
      </AlertDialog>
    </>
  );
}
