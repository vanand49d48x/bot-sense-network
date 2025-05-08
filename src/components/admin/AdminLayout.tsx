
import { useAuth } from "@/context/AuthContext";
import { Navigate, Outlet } from "react-router-dom";
import { useToast } from "@/hooks/use-toast";
import { Loader2 } from "lucide-react";
import { AdminSidebar } from "./AdminSidebar";

export function AdminLayout() {
  const { user, loading, isAdmin } = useAuth();
  const { toast } = useToast();

  // Show loading state
  if (loading) {
    return (
      <div className="flex items-center justify-center min-h-screen">
        <Loader2 className="h-8 w-8 animate-spin text-primary" />
        <span className="ml-2">Verifying admin access...</span>
      </div>
    );
  }

  // If user is not logged in, redirect to auth
  if (!user) {
    return <Navigate to="/auth?returnUrl=/admin" replace />;
  }

  // If user is logged in but not admin, show unauthorized
  if (!isAdmin) {
    toast({
      title: "Unauthorized",
      description: "You do not have access to the admin area",
      variant: "destructive",
    });
    return <Navigate to="/dashboard" replace />;
  }

  // If user is admin, show admin layout
  return (
    <div className="flex min-h-screen">
      <AdminSidebar />
      <div className="flex-1 p-6 overflow-auto">
        <Outlet />
      </div>
    </div>
  );
}
