
import { SidebarProvider, SidebarTrigger } from "@/components/ui/sidebar";
import { AppSidebar } from "./AppSidebar";
import { Button } from "@/components/ui/button";
import { LogOut, Mail, FileText, Book, Calendar, Github, Twitter, Linkedin, Info } from "lucide-react";
import { useAuth } from "@/context/AuthContext";
import { useNavigate } from "react-router-dom";
import { ThemeToggle } from "./ThemeToggle";
import { Link } from "react-router-dom";
import { Separator } from "@/components/ui/separator";

interface MainLayoutProps {
  children: React.ReactNode;
}

export function MainLayout({ children }: MainLayoutProps) {
  const { signOut } = useAuth();
  const navigate = useNavigate();

  const handleSignOut = async () => {
    await signOut();
    navigate("/");
  };

  return (
    <SidebarProvider>
      <div className="min-h-screen flex w-full flex-col">
        <div className="flex flex-1 w-full">
          <AppSidebar />
          <div className="flex-1 flex flex-col min-h-screen">
            <header className="border-b border-border/40 p-4 flex justify-between items-center">
              <SidebarTrigger />
              <div className="flex items-center gap-2">
                <ThemeToggle />
                <Button variant="ghost" size="sm" onClick={handleSignOut} className="flex items-center gap-2">
                  <LogOut size={16} />
                  <span>Sign Out</span>
                </Button>
              </div>
            </header>
            <main className="flex-1 p-4 md:p-6 overflow-auto">
              {children}
            </main>
          </div>
        </div>
        
        {/* Footer */}
        <footer className="border-t border-border/40 bg-muted/30 py-8 px-4 md:px-6">
          <div className="container mx-auto">
            <div className="grid grid-cols-1 gap-8 md:grid-cols-2 lg:grid-cols-5">
              {/* About */}
              <div>
                <h3 className="text-lg font-semibold mb-3 flex items-center gap-2">
                  <Info size={16} className="text-primary" />
                  About
                </h3>
                <ul className="space-y-2 text-sm">
                  <li><Link to="/about" className="hover:text-primary transition-colors">About Us</Link></li>
                  <li><Link to="/contact" className="hover:text-primary transition-colors">Contact</Link></li>
                  <li><span className="text-muted-foreground">Blog (Coming Soon)</span></li>
                </ul>
              </div>
              
              {/* Legal */}
              <div>
                <h3 className="text-lg font-semibold mb-3 flex items-center gap-2">
                  <FileText size={16} className="text-primary" />
                  Legal
                </h3>
                <ul className="space-y-2 text-sm">
                  <li><Link to="/privacy" className="hover:text-primary transition-colors">Privacy Policy</Link></li>
                  <li><Link to="/terms" className="hover:text-primary transition-colors">Terms of Service</Link></li>
                  <li><Link to="/risk-disclosure" className="hover:text-primary transition-colors">Risk Disclosure</Link></li>
                  <li><Link to="/cookies" className="hover:text-primary transition-colors">Cookie Policy</Link></li>
                  <li><Link to="/security" className="hover:text-primary transition-colors">Security Measures</Link></li>
                  <li><Link to="/disclaimer" className="hover:text-primary transition-colors">Disclaimer</Link></li>
                  <li><Link to="/regulatory" className="hover:text-primary transition-colors">Regulatory Information</Link></li>
                </ul>
              </div>
              
              {/* Product */}
              <div>
                <h3 className="text-lg font-semibold mb-3 flex items-center gap-2">
                  <Book size={16} className="text-primary" />
                  Product
                </h3>
                <ul className="space-y-2 text-sm">
                  <li><Link to="/features" className="hover:text-primary transition-colors">Features</Link></li>
                  <li><Link to="/api-docs" className="hover:text-primary transition-colors">API Docs</Link></li>
                  <li><Link to="/integration" className="hover:text-primary transition-colors">Integration Guide</Link></li>
                  <li><Link to="/status" className="hover:text-primary transition-colors">
                    <span className="flex items-center gap-1">
                      Status Page
                      <span className="bg-green-500 text-white text-xs px-1 rounded">Online</span>
                    </span>
                  </Link></li>
                </ul>
              </div>
              
              {/* Company */}
              <div>
                <h3 className="text-lg font-semibold mb-3">Company</h3>
                <div className="space-y-4">
                  <div>
                    <p className="text-sm mb-2">Subscribe to our newsletter</p>
                    <div className="flex gap-2">
                      <input 
                        type="email" 
                        placeholder="Your email"
                        className="bg-background text-sm px-3 py-2 rounded border border-border flex-1"
                      />
                      <Button size="sm">Subscribe</Button>
                    </div>
                  </div>
                  <div>
                    <p className="text-sm mb-2">Follow us</p>
                    <div className="flex gap-3">
                      <a href="https://github.com/robometrics" target="_blank" rel="noopener noreferrer" 
                         className="hover:text-primary transition-colors">
                        <Github size={20} />
                      </a>
                      <a href="https://twitter.com/robometrics" target="_blank" rel="noopener noreferrer" 
                         className="hover:text-primary transition-colors">
                        <Twitter size={20} />
                      </a>
                      <a href="https://linkedin.com/company/robometrics" target="_blank" rel="noopener noreferrer" 
                         className="hover:text-primary transition-colors">
                        <Linkedin size={20} />
                      </a>
                    </div>
                  </div>
                </div>
              </div>
              
              {/* Contact */}
              <div>
                <h3 className="text-lg font-semibold mb-3">Contact</h3>
                <div className="space-y-2 text-sm">
                  <div className="flex items-center gap-2">
                    <Mail size={16} className="text-primary" />
                    <a href="mailto:support@robometrics.io" className="hover:text-primary transition-colors">
                      support@robometrics.io
                    </a>
                  </div>
                </div>
              </div>
            </div>
            
            <Separator className="my-6" />
            
            <div className="flex flex-col md:flex-row justify-between items-center gap-4">
              <div className="flex items-center gap-2">
                <span className="font-semibold">RoboMetrics</span>
              </div>
              <p className="text-sm text-muted-foreground">
                © {new Date().getFullYear()} RoboMetrics. All rights reserved.
              </p>
            </div>
          </div>
        </footer>
      </div>
    </SidebarProvider>
  );
}
