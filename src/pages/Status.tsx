
import React, { useEffect } from "react";
import PlaceholderLayout from "@/components/layout/PlaceholderLayout";
import { Card, CardContent } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";

const Status = () => {
  useEffect(() => {
    // Scroll to the top of the page when component mounts
    window.scrollTo(0, 0);
  }, []);
  
  return (
    <PlaceholderLayout title="System Status">
      <div className="flex items-center gap-2 mb-8">
        <Badge className="bg-green-500">All Systems Operational</Badge>
        <span className="text-muted-foreground">Updated 5 minutes ago</span>
      </div>
      
      <div className="grid gap-4">
        <Card>
          <CardContent className="p-4 flex justify-between items-center">
            <span>API Endpoints</span>
            <Badge variant="outline" className="text-green-500 border-green-500">Operational</Badge>
          </CardContent>
        </Card>
        
        <Card>
          <CardContent className="p-4 flex justify-between items-center">
            <span>Dashboard</span>
            <Badge variant="outline" className="text-green-500 border-green-500">Operational</Badge>
          </CardContent>
        </Card>
        
        <Card>
          <CardContent className="p-4 flex justify-between items-center">
            <span>Authentication Services</span>
            <Badge variant="outline" className="text-green-500 border-green-500">Operational</Badge>
          </CardContent>
        </Card>
        
        <Card>
          <CardContent className="p-4 flex justify-between items-center">
            <span>Telemetry Ingestion</span>
            <Badge variant="outline" className="text-green-500 border-green-500">Operational</Badge>
          </CardContent>
        </Card>
      </div>
    </PlaceholderLayout>
  );
};

export default Status;
