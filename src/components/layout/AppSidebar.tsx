import { Link, useLocation } from "react-router-dom";
import {
  Sidebar,
  SidebarContent,
  SidebarGroup,
  SidebarGroupContent,
  SidebarGroupLabel,
  SidebarHeader,
  SidebarMenu,
  SidebarMenuButton,
  SidebarMenuItem,
  SidebarMenuSub,
  SidebarMenuSubButton,
  SidebarMenuSubItem
} from "@/components/ui/sidebar";
import { Battery, MapPin, Bell, ArrowRight, Key, BarChart3, UserCog, Link2, CreditCard, Settings } from "lucide-react";
import { ApiKeySettings } from "./ApiKeySettings";
import { useAuth } from "@/context/AuthContext";
import { useState, useEffect } from "react";

export function AppSidebar() {
  const { session } = useAuth();
  const location = useLocation();
  const [settingsOpen, setSettingsOpen] = useState(false);
  
  // Keep settings menu open when on settings pages
  useEffect(() => {
    if (location.pathname.startsWith("/settings/")) {
      setSettingsOpen(true);
    }
  }, [location.pathname]);
  
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
              
              {/* Settings dropdown menu */}
              <SidebarMenuItem>
                <SidebarMenuButton 
                  onClick={() => setSettingsOpen(!settingsOpen)} 
                  data-state={settingsOpen ? "open" : "closed"}
                  isActive={location.pathname.startsWith("/settings/")}
                >
                  <Settings className="h-5 w-5" />
                  <span>Settings</span>
                </SidebarMenuButton>
                {settingsOpen && (
                  <SidebarMenuSub>
                    <SidebarMenuSubItem>
                      <SidebarMenuSubButton 
                        asChild 
                        isActive={location.pathname === "/settings/profile"}
                      >
                        <Link to="/settings/profile">
                          <UserCog className="h-4 w-4" />
                          <span>Profile</span>
                        </Link>
                      </SidebarMenuSubButton>
                    </SidebarMenuSubItem>
                    <SidebarMenuSubItem>
                      <SidebarMenuSubButton 
                        asChild
                        isActive={location.pathname === "/settings/subscription"}
                      >
                        <Link to="/settings/subscription">
                          <CreditCard className="h-4 w-4" />
                          <span>Subscription</span>
                        </Link>
                      </SidebarMenuSubButton>
                    </SidebarMenuSubItem>
                  </SidebarMenuSub>
                )}
              </SidebarMenuItem>
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
