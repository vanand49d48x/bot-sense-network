
import React from "react";
import { Shield } from "lucide-react";
import { Separator } from "@/components/ui/separator";

const SecurityMeasures = () => {
  return (
    <div className="max-w-4xl mx-auto py-12 px-4">
      <div className="flex items-center gap-2 mb-6">
        <Shield className="h-8 w-8 text-primary" />
        <h1 className="text-3xl font-bold">Security Measures</h1>
      </div>
      <p className="text-muted-foreground mb-8">Effective Date: April 28, 2025</p>
      
      <p className="mb-6">RoboMetrics uses:</p>
      
      <ul className="list-disc pl-8 mb-8 space-y-3">
        <li>HTTPS with TLS 1.3 encryption</li>
        <li>Supabase-secured Postgres with Row-Level Security</li>
        <li>Role-based access for internal dashboards</li>
        <li>API key or token-based robot authentication</li>
      </ul>
      
      <p className="mb-6">We actively monitor for anomalies and breaches.</p>
      
      <Separator className="my-10" />
      
      <p className="text-sm text-muted-foreground">
        If you have any questions about our security measures, please contact us at{" "}
        <a href="mailto:support@robometrics.io" className="text-primary">
          support@robometrics.io
        </a>
      </p>
    </div>
  );
};

export default SecurityMeasures;
