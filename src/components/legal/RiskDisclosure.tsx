
import React from "react";
import { Separator } from "@/components/ui/separator";

const RiskDisclosure = () => {
  return (
    <div className="max-w-4xl mx-auto py-12 px-4">
      <h1 className="text-3xl font-bold mb-6">Risk Disclosure</h1>
      <p className="text-muted-foreground mb-6">Effective Date: April 28, 2025</p>
      
      <p className="mb-6">
        Monitoring data is provided as-is. It may not represent real-time physical robot states. 
        You should not rely on RoboMetrics data as your sole source of safety or operational decision-making.
      </p>
      
      <p className="mb-6">
        We disclaim liability for damages arising from the use or failure of the RoboMetrics system.
      </p>
      
      <Separator className="my-10" />
      
      <p className="text-sm text-muted-foreground">
        If you have any questions about our risk disclosure, please contact us at{" "}
        <a href="mailto:support@robometrics.io" className="text-primary">
          support@robometrics.io
        </a>
      </p>
    </div>
  );
};

export default RiskDisclosure;
