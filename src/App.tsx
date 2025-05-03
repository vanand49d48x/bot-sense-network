
import { Toaster } from "@/components/ui/toaster";
import { Toaster as Sonner } from "@/components/ui/sonner";
import { TooltipProvider } from "@/components/ui/tooltip";
import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
import { BrowserRouter, Routes, Route, Navigate } from "react-router-dom";
import { AuthProvider, useAuth } from "@/context/AuthContext";
import { ThemeProvider } from "@/context/ThemeContext";
import CookieConsent from "@/components/legal/CookieConsent";
import Index from "./pages/Index";
import Auth from "./pages/Auth";
import NotFound from "./pages/NotFound";
import IntegrationGuide from "./pages/IntegrationGuide";
import Landing from "./pages/Landing";
import MapViewPage from "./pages/MapViewPage";
import Alerts from "./pages/Alerts";
import FleetStatusPage from "./pages/FleetStatusPage";
import ProfilePage from "./pages/ProfilePage";
import Pricing from "./pages/Pricing";

// Legal pages
import Privacy from "./pages/legal/Privacy";
import Terms from "./pages/legal/Terms";
import RiskPage from "./pages/legal/RiskPage";
import Cookies from "./pages/legal/Cookies";
import Security from "./pages/legal/Security";
import DisclaimerPage from "./pages/legal/DisclaimerPage";
import Regulatory from "./pages/legal/Regulatory";
import Contact from "./pages/Contact";

// New pages
import About from "./pages/About";
import Features from "./pages/Features";
import ApiDocs from "./pages/ApiDocs";
import Status from "./pages/Status";
import Blog from "./pages/Blog";

const queryClient = new QueryClient();

// Protected route component
const ProtectedRoute = ({ children }: { children: React.ReactNode }) => {
  const { user, loading } = useAuth();
  
  if (loading) {
    return <div className="flex min-h-screen items-center justify-center">Loading...</div>;
  }
  
  if (!user) {
    return <Navigate to="/auth" replace />;
  }
  
  return <>{children}</>;
};

const App = () => (
  <QueryClientProvider client={queryClient}>
    <ThemeProvider>
      <AuthProvider>
        <TooltipProvider>
          <Toaster />
          <Sonner />
          <CookieConsent />
          <BrowserRouter>
            <Routes>
              <Route path="/" element={<Landing />} />
              <Route path="/auth" element={<Auth />} />
              <Route path="/pricing" element={<Pricing />} />
              <Route 
                path="/dashboard" 
                element={
                  <ProtectedRoute>
                    <Index />
                  </ProtectedRoute>
                } 
              />
              <Route 
                path="/fleet-status" 
                element={
                  <ProtectedRoute>
                    <FleetStatusPage />
                  </ProtectedRoute>
                }
              />
              <Route 
                path="/map" 
                element={
                  <ProtectedRoute>
                    <MapViewPage />
                  </ProtectedRoute>
                }
              />
              <Route 
                path="/alerts" 
                element={
                  <ProtectedRoute>
                    <Alerts />
                  </ProtectedRoute>
                }
              />
              <Route 
                path="/profile" 
                element={
                  <ProtectedRoute>
                    <ProfilePage />
                  </ProtectedRoute>
                }
              />
              <Route 
                path="/integration" 
                element={
                  <ProtectedRoute>
                    <IntegrationGuide />
                  </ProtectedRoute>
                }
              />
              
              {/* Legal Routes */}
              <Route path="/privacy" element={<Privacy />} />
              <Route path="/terms" element={<Terms />} />
              <Route path="/risk-disclosure" element={<RiskPage />} />
              <Route path="/cookies" element={<Cookies />} />
              <Route path="/security" element={<Security />} />
              <Route path="/disclaimer" element={<DisclaimerPage />} />
              <Route path="/regulatory" element={<Regulatory />} />
              <Route path="/contact" element={<Contact />} />
              
              {/* New Routes */}
              <Route path="/about" element={<About />} />
              <Route path="/features" element={<Features />} />
              <Route path="/api-docs" element={<ApiDocs />} />
              <Route path="/status" element={<Status />} />
              <Route path="/blog" element={<Blog />} />
              
              <Route path="*" element={<NotFound />} />
            </Routes>
          </BrowserRouter>
        </TooltipProvider>
      </AuthProvider>
    </ThemeProvider>
  </QueryClientProvider>
);

export default App;
