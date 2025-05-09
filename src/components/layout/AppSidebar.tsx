
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
import { Battery, MapPin, Bell, ArrowRight, Key, BarChart3, UserCog, Link2 } from "lucide-react";
import { ApiKeySettings } from "./ApiKeySettings";
import { useAuth } from "@/context/AuthContext";

export function AppSidebar() {
  const { session } = useAuth();
  
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
      </SidebarContent>
    </Sidebar>
  );
}
