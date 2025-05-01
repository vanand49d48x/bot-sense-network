
import { BrowserRouter as Router, Routes, Route } from "react-router-dom";
import { ThemeProvider } from "@/components/theme-provider";
import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
import { TooltipProvider } from "@/components/ui/tooltip";
import { Toaster } from "@/components/ui/sonner";
import { AuthProvider } from "@/context/AuthContext";

// Pages
import Index from "@/pages/Index";
import Landing from "@/pages/Landing";
import Auth from "@/pages/Auth";
import IntegrationGuide from "@/pages/IntegrationGuide";
import NotFound from "@/pages/NotFound";
import FleetStatus from "@/pages/FleetStatus";
import MapViewPage from "@/pages/MapViewPage";
import Alerts from "@/pages/Alerts";

// Create a client
const queryClient = new QueryClient();

function App() {
  return (
    <ThemeProvider attribute="class" defaultTheme="system" enableSystem storageKey="theme">
      <QueryClientProvider client={queryClient}>
        <AuthProvider>
          <TooltipProvider>
            <Router>
              <Routes>
                <Route path="/" element={<Landing />} />
                <Route path="/auth" element={<Auth />} />
                <Route path="/dashboard" element={<Index />} />
                <Route path="/fleet-status" element={<FleetStatus />} />
                <Route path="/map-view" element={<MapViewPage />} />
                <Route path="/alerts" element={<Alerts />} />
                <Route path="/integration" element={<IntegrationGuide />} />
                <Route path="*" element={<NotFound />} />
              </Routes>
            </Router>
            <Toaster />
          </TooltipProvider>
        </AuthProvider>
      </QueryClientProvider>
    </ThemeProvider>
  );
}

export default App;
