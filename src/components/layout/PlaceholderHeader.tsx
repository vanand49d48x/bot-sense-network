
import { Link, useNavigate } from "react-router-dom";
import { Button } from "@/components/ui/button";
import { ThemeToggle } from "./ThemeToggle";
import { useAuth } from "@/context/AuthContext";
import { useEffect, useState } from "react";

export default function PlaceholderHeader() {
  const { user, session } = useAuth();
  const [isAuthenticated, setIsAuthenticated] = useState<boolean>(false);
  const navigate = useNavigate();
  
  useEffect(() => {
    // Update authentication status whenever user or session changes
    setIsAuthenticated(!!user && !!session);
  }, [user, session]);

  return (
    <header className="supports-backdrop-blur:bg-background/60 sticky top-0 z-40 w-full backdrop-blur border-b bg-background/95">
      <div className="container flex h-14 items-center">
        <div className="mr-4 flex">
          <Link to="/" className="flex items-center space-x-2 mr-6">
            <span className="font-bold text-xl">RoboMetrics</span>
          </Link>
        </div>
        <div className="hidden md:flex flex-1 items-center justify-between">
          <nav className="flex items-center space-x-6 text-sm font-medium">
            <Link to="/about" className="transition-colors hover:text-foreground/80">
              About
            </Link>
            <Link to="/features" className="transition-colors hover:text-foreground/80">
              Features
            </Link>
            <Link to="/pricing" className="transition-colors hover:text-foreground/80">
              Pricing
            </Link>
            <Link to="/blog" className="transition-colors hover:text-foreground/80">
              Blog
            </Link>
            <Link to="/contact" className="transition-colors hover:text-foreground/80">
              Contact
            </Link>
          </nav>
          <div className="flex items-center gap-4">
            <ThemeToggle />
            {isAuthenticated ? (
              <Link to="/dashboard">
                <Button size="sm">Dashboard</Button>
              </Link>
            ) : (
              <Link to="/auth">
                <Button size="sm">Sign In</Button>
              </Link>
            )}
          </div>
        </div>
        <div className="flex flex-1 items-center justify-end space-x-4 md:hidden">
          <ThemeToggle />
          {isAuthenticated ? (
            <Link to="/dashboard">
              <Button size="sm">Dashboard</Button>
            </Link>
          ) : (
            <Link to="/auth">
              <Button size="sm">Sign In</Button>
            </Link>
          )}
        </div>
      </div>
    </header>
  );
}
