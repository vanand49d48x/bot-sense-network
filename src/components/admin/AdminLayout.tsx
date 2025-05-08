
import { useAuth } from "@/context/AuthContext";
import { Navigate, Outlet } from "react-router-dom";
import { useToast } from "@/hooks/use-toast";
import { Loader2 } from "lucide-react";
import { AdminSidebar } from "./AdminSidebar";
import { useEffect, useState } from "react";

export function AdminLayout() {
  const { user, loading: authLoading, isAdmin } = useAuth();
  const [isReady, setIsReady] = useState(false);
  const { toast } = useToast();
  
  // Control component mounting
  useEffect(() => {
    // Add a short delay to ensure all auth checks are completed
    const timer = setTimeout(() => {
      setIsReady(true);
    }, 100);
    
    return () => clearTimeout(timer);
  }, [user, isAdmin, authLoading]);

  // Don't render anything until auth is done loading and component is ready
  if (authLoading || !isReady) {
    return (
      <div className="flex items-center justify-center min-h-screen">
        <Loader2 className="h-8 w-8 animate-spin text-primary" />
        <span className="ml-2">Loading authentication...</span>
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
