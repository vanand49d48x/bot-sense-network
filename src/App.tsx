import {
  BrowserRouter as Router,
  Route,
  Routes,
} from "react-router-dom";
import { ThemeProvider } from "@/components/theme-provider"
import { cn } from "@/lib/utils"
import { useToast } from "@/components/ui/use-toast"
import { Toaster } from "@/components/ui/toaster"
import { Landing } from "@/pages/Landing";
import { Auth } from "@/pages/Auth";
import NotFound from "@/pages/NotFound";
import Index from "@/pages/Index";
import FleetStatus from "@/pages/FleetStatus";
import Alerts from "@/pages/Alerts";
import MapViewPage from "@/pages/MapViewPage";
import IntegrationGuide from "@/pages/IntegrationGuide";
import RobotDetails from "./pages/RobotDetails";

function App() {
  const { toast } = useToast()

  return (
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
          <Route path="/robot/:robotId" element={<RobotDetails />} /> {/* New route */}
          <Route path="/fleet" element={<FleetStatus />} />
          <Route path="/alerts" element={<Alerts />} />
          <Route path="/map" element={<MapViewPage />} />
          <Route path="/integration-guide" element={<IntegrationGuide />} />
          <Route path="*" element={<NotFound />} />
        </Routes>
      </Router>
      <Toaster />
    </ThemeProvider>
  );
}

export default App;
