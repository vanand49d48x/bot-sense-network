
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

export function AppSidebar() {
  const { session } = useAuth();
  const [isAdmin, setIsAdmin] = useState(false);

  useEffect(() => {
    checkAdminStatus();
  }, [session]);

  const checkAdminStatus = async () => {
    if (!session?.user) return;

    try {
      const { data, error } = await supabase
        .from('admin_users')
        .select('*')
        .eq('id', session.user.id)
        .single();

      if (error) throw error;
      setIsAdmin(!!data);
    } catch (error) {
      console.error('Error checking admin status:', error);
      setIsAdmin(false);
    }
  };
  
  const mainMenuItems = [
    {
      title: "Dashboard",
      icon: ArrowRight,
      url: "/dashboard",
      requiresAuth: true,
    },
    {
      title: "Fleet Status",
      icon: BarChart3,
      url: "/fleet-status",
      requiresAuth: true,
    },
    {
      title: "Map View",
      icon: MapPin,
      url: "/map",
      requiresAuth: true,
    },
    {
      title: "Alerts",
      icon: Bell,
      url: "/alerts",
      requiresAuth: true,
    },
    {
      title: "Integration",
      icon: Link2,
      url: "/integration",
      requiresAuth: false,
    },
    {
      title: "API Docs",
      icon: Key,
      url: "/api-docs",
      requiresAuth: false,
    },
    {
      title: "Profile",
      icon: UserCog,
      url: "/profile",
      requiresAuth: true,
    }
  ];

  // Add admin link if user is admin
  if (isAdmin) {
    mainMenuItems.push({
      title: "Admin",
      icon: Shield,
      url: "/admin",
      requiresAuth: true,
    });
  }

  // Filter items based on authentication status
  const visibleMenuItems = mainMenuItems.filter(item => 
    !item.requiresAuth || (item.requiresAuth && session?.user)
  );

  return (
    <Sidebar>
      <SidebarHeader className="py-6 px-4 border-b border-sidebar-border">
        <Link to="/">
          <h1 className="text-xl font-bold">RoboMetrics</h1>
        </Link>
      </SidebarHeader>
      <SidebarContent>
        <SidebarGroup>
          <SidebarGroupLabel>Monitoring</SidebarGroupLabel>
          <SidebarGroupContent>
            <SidebarMenu>
              {visibleMenuItems.map((item) => (
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
