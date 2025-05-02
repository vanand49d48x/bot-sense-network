
import { Button } from "@/components/ui/button";
import { useTheme } from "@/context/ThemeContext";
import { Moon, Sun } from "lucide-react";

export function ThemeToggle() {
  const { theme, toggleTheme } = useTheme();
  
  return (
    <Button 
      variant="ghost" 
      size="icon" 
      onClick={toggleTheme} 
      className="rounded-full"
      aria-label="Toggle theme"
    >
      {theme === "light" ? (
        <Moon size={20} className="transition-all" />
      ) : (
        <Sun size={20} className="transition-all" />
      )}
      <span className="sr-only">{theme === "light" ? "Dark mode" : "Light mode"}</span>
    </Button>
  );
}
