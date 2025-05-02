
import React, { useEffect } from "react";
import PlaceholderLayout from "@/components/layout/PlaceholderLayout";
import { AlertTriangle, Battery, MapPin, Activity } from "lucide-react";
import { Card, CardHeader, CardTitle, CardContent } from "@/components/ui/card";

const Features = () => {
  useEffect(() => {
    // Scroll to the top of the page when component mounts
    window.scrollTo(0, 0);
  }, []);
  
  return (
    <PlaceholderLayout title="Features">
      <p className="text-muted-foreground mb-8">
        Discover how RoboMetrics can help you keep track of your robots in real-time.
      </p>
      
      <div className="grid grid-cols-1 md:grid-cols-2 gap-6 mb-12">
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <Battery className="h-5 w-5 text-primary" />
              Battery Monitoring
            </CardTitle>
          </CardHeader>
          <CardContent>
            <p>Real-time battery level tracking and low battery alerts.</p>
          </CardContent>
        </Card>
        
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <MapPin className="h-5 w-5 text-primary" />
              Location Tracking
            </CardTitle>
          </CardHeader>
          <CardContent>
            <p>Monitor robot positions with our interactive map view.</p>
          </CardContent>
        </Card>
        
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <Activity className="h-5 w-5 text-primary" />
              Health Monitoring
            </CardTitle>
          </CardHeader>
          <CardContent>
            <p>Track vitals and receive alerts when robots need maintenance.</p>
          </CardContent>
        </Card>
        
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <AlertTriangle className="h-5 w-5 text-primary" />
              Custom Alerts
            </CardTitle>
          </CardHeader>
          <CardContent>
            <p>Set up custom alerts for your specific needs.</p>
          </CardContent>
        </Card>
      </div>
      
      <p className="text-center text-muted-foreground">
        More features coming soon!
      </p>
    </PlaceholderLayout>
  );
};

export default Features;
