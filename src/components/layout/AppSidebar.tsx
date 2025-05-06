
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
import { Battery, MapPin, Bell, ArrowRight, Key, BarChart3, UserCog, Link2, CreditCard } from "lucide-react";
import { ApiKeySettings } from "./ApiKeySettings";
import { useAuth } from "@/context/AuthContext";
import { useSubscription } from "@/context/SubscriptionContext";

export function AppSidebar() {
  const { session } = useAuth();
  const { subscriptionTier, isSubscriptionActive } = useSubscription();
  
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
    },
    {
      title: "Subscription",
      icon: CreditCard,
      url: "/subscription-manage",
    }
  ];

  return (
    <Sidebar>
      <SidebarHeader className="py-6 px-4 border-b border-sidebar-border">
        <div className="flex flex-col">
          <h1 className="text-xl font-bold">RoboMetrics</h1>
          {isSubscriptionActive && (
            <div className="mt-1 text-xs text-muted-foreground flex items-center">
              <span className="bg-primary/20 text-primary rounded-full px-2 py-0.5">
                {subscriptionTier.charAt(0).toUpperCase() + subscriptionTier.slice(1)} Plan
              </span>
            </div>
          )}
        </div>
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
