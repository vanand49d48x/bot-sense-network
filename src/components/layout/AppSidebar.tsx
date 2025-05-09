
import { Link } from "react-router-dom";
import {
  Sidebar,
  SidebarContent,
  SidebarGroup,
  SidebarGroupContent,
  SidebarGroupLabel,
  SidebarHeader,
  SidebarMenu,
  SidebarMenuButton,
  SidebarMenuItem
} from "@/components/ui/sidebar";
import { Battery, MapPin, Bell, ArrowRight, Key, BarChart3, UserCog, Link2, Shield } from "lucide-react";
import { ApiKeySettings } from "./ApiKeySettings";
import { useAuth } from "@/context/AuthContext";
import { useEffect, useState } from "react";
import { supabase } from "@/integrations/supabase/client";
import { useToast } from "@/hooks/use-toast";

export function AppSidebar() {
  const { session } = useAuth();
  const { toast } = useToast();
  const [isAdmin, setIsAdmin] = useState(false);
  const [isCheckingAdmin, setIsCheckingAdmin] = useState(false);

  useEffect(() => {
    if (session?.user) {
      checkAdminStatus();
    } else {
      setIsAdmin(false);
    }
  }, [session]);

  const checkAdminStatus = async () => {
    if (!session?.user) return;

    try {
      setIsCheckingAdmin(true);
      // Use the is_admin security definer function to safely check admin status
      const { data, error } = await supabase
        .rpc('is_admin', { user_id: session.user.id });

      if (error) {
        console.error('Error checking admin status:', error);
        toast({
          title: "Error checking admin status",
          description: error.message,
          variant: "destructive",
        });
        setIsAdmin(false);
      } else {
        setIsAdmin(!!data);
      }
    } catch (error: any) {
      console.error('Error in admin status check:', error);
      toast({
        title: "Error in admin status check",
        description: error.message,
        variant: "destructive",
      });
      setIsAdmin(false);
    } finally {
      setIsCheckingAdmin(false);
    }
  };
  
  const mainMenuItems = [
    {
      title: "Dashboard",
      icon: ArrowRight,
      url: "/dashboard",
    },
    {
      title: "Fleet Status",
      icon: BarChart3,
      url: "/fleet-status",
    },
    {
      title: "Map View",
      icon: MapPin,
      url: "/map",
    },
    {
      title: "Alerts",
      icon: Bell,
      url: "/alerts",
    },
    {
      title: "Integration",
      icon: Link2,
      url: "/integration",
    },
    {
      title: "Profile",
      icon: UserCog,
      url: "/profile",
    }
  ];

  // Add admin link if user is admin
  if (isAdmin) {
    mainMenuItems.push({
      title: "Admin",
      icon: Shield,
      url: "/admin",
    });
  }

  return (
    <Sidebar>
      <SidebarHeader className="py-6 px-4 border-b border-sidebar-border">
        <h1 className="text-xl font-bold">RoboMetrics</h1>
      </SidebarHeader>
      <SidebarContent>
        <SidebarGroup>
          <SidebarGroupLabel>Monitoring</SidebarGroupLabel>
          <SidebarGroupContent>
            <SidebarMenu>
              {mainMenuItems.map((item) => (
                <SidebarMenuItem key={item.title}>
                  <SidebarMenuButton asChild>
                    <Link to={item.url} className="flex items-center space-x-2">
                      <item.icon className="h-5 w-5" />
                      <span>{item.title}</span>
                    </Link>
                  </SidebarMenuButton>
                </SidebarMenuItem>
              ))}
            </SidebarMenu>
          </SidebarGroupContent>
        </SidebarGroup>
        
        {session && (
          <SidebarGroup className="mt-6">
            <SidebarGroupLabel className="flex items-center gap-1">
              <Key className="h-4 w-4" />
              <span>API Integration</span>
            </SidebarGroupLabel>
            <SidebarGroupContent>
              <div className="p-2">
                <ApiKeySettings />
              </div>
            </SidebarGroupContent>
          </SidebarGroup>
        )}
      </SidebarContent>
    </Sidebar>
  );
}
