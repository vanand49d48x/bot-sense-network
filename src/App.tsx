
import {
  BrowserRouter as Router,
  Route,
  Routes,
} from "react-router-dom";
import { ThemeProvider } from "@/components/theme-provider"
import { Toaster } from "@/components/ui/toaster"
import Landing from "@/pages/Landing";
import Auth from "@/pages/Auth";
import NotFound from "@/pages/NotFound";
import Index from "@/pages/Index";
import FleetStatus from "@/pages/FleetStatus";
import Alerts from "@/pages/Alerts";
import MapViewPage from "@/pages/MapViewPage";
import IntegrationGuide from "@/pages/IntegrationGuide";
import RobotDetails from "./pages/RobotDetails";
import { AuthProvider } from "./context/AuthContext";
import { QueryClient, QueryClientProvider } from "@tanstack/react-query";

// Create a client
const queryClient = new QueryClient();

function App() {
  return (
    <QueryClientProvider client={queryClient}>
      <AuthProvider>
        <ThemeProvider
          attribute="class"
          defaultTheme="system"
          enableSystem
          disableTransitionOnChange
        >
          <Router>
            <Routes>
              <Route path="/" element={<Landing />} />
              <Route path="/auth" element={<Auth />} />
              <Route path="/dashboard" element={<Index />} />
              <Route path="/robot/:robotId" element={<RobotDetails />} />
              <Route path="/fleet-status" element={<FleetStatus />} />
              <Route path="/alerts" element={<Alerts />} />
              <Route path="/map-view" element={<MapViewPage />} />
              <Route path="/integration-guide" element={<IntegrationGuide />} />
              <Route path="*" element={<NotFound />} />
            </Routes>
          </Router>
          <Toaster />
        </ThemeProvider>
      </AuthProvider>
    </QueryClientProvider>
  );
}

export default App;
