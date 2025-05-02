
import { Toaster } from "@/components/ui/toaster";
import { Toaster as Sonner } from "@/components/ui/sonner";
import { TooltipProvider } from "@/components/ui/tooltip";
import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
import { BrowserRouter, Routes, Route, Navigate } from "react-router-dom";
import { AuthProvider, useAuth } from "@/context/AuthContext";
import { ThemeProvider } from "@/context/ThemeContext";
import Index from "./pages/Index";
import Auth from "./pages/Auth";
import NotFound from "./pages/NotFound";
import IntegrationGuide from "./pages/IntegrationGuide";
import Landing from "./pages/Landing";
import MapViewPage from "./pages/MapViewPage";
import Alerts from "./pages/Alerts";
import FleetStatusPage from "./pages/FleetStatusPage";
import ProfilePage from "./pages/ProfilePage";

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
          <BrowserRouter>
            <Routes>
              <Route path="/" element={<Landing />} />
              <Route path="/auth" element={<Auth />} />
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
              <Route path="*" element={<NotFound />} />
            </Routes>
          </BrowserRouter>
        </TooltipProvider>
      </AuthProvider>
    </ThemeProvider>
  </QueryClientProvider>
);

export default App;
