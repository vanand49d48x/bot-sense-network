
import { Link, useLocation } from "react-router-dom";
import { buttonVariants } from "@/components/ui/button";
import { cn } from "@/lib/utils";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import {
  Users,
  Bot,
  BarChart2,
  Settings,
  Home,
  LogOut,
} from "lucide-react";
import { useAuth } from "@/context/AuthContext";
import { useNavigate } from "react-router-dom";
import { ThemeToggle } from "@/components/layout/ThemeToggle";

export function AdminSidebar() {
  const { signOut } = useAuth();
  const location = useLocation();
  const navigate = useNavigate();

  const navigation = [
    {
      title: "Overview",
      href: "/admin",
      icon: Home,
    },
    {
      title: "User Management",
      href: "/admin/users",
      icon: Users,
    },
    {
      title: "Robot Fleet",
      href: "/admin/robots",
      icon: Bot,
    },
    {
      title: "Analytics",
      href: "/admin/analytics",
      icon: BarChart2,
    },
    {
      title: "System Settings",
      href: "/admin/settings",
      icon: Settings,
    },
  ];

  const handleSignOut = async () => {
    await signOut();
    navigate("/");
  };

  return (
    <div className="flex flex-col border-r w-64 min-h-screen">
      <div className="p-4">
        <h2 className="text-xl font-bold mb-2 flex items-center">
          <Bot className="mr-2 h-5 w-5 text-primary" />
          Admin Dashboard
        </h2>
        <div className="text-sm text-muted-foreground">
          System Administration
        </div>
      </div>

      <Separator />

      <ScrollArea className="flex-1 px-2">
        <div className="space-y-1 py-4">
          {navigation.map((item) => (
            <Link
              key={item.href}
              to={item.href}
              className={cn(
                buttonVariants({ variant: "ghost" }),
                location.pathname === item.href
                  ? "bg-muted hover:bg-muted"
                  : "hover:bg-transparent hover:underline",
                "justify-start w-full flex items-center"
              )}
            >
              <item.icon className="mr-2 h-4 w-4" />
              {item.title}
            </Link>
          ))}
        </div>
      </ScrollArea>

      <Separator />

      <div className="p-4 flex flex-col space-y-2">
        <Link
          to="/dashboard"
          className={buttonVariants({ variant: "outline", size: "sm" })}
        >
          Return to Dashboard
        </Link>
        <div className="flex items-center justify-between">
          <button
            onClick={handleSignOut}
            className={cn(
              buttonVariants({ variant: "ghost", size: "sm" }),
              "flex items-center text-red-500"
            )}
          >
            <LogOut className="h-4 w-4 mr-2" /> Sign out
          </button>
          <ThemeToggle />
        </div>
      </div>
    </div>
  );
}
