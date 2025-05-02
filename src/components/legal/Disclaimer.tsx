
import React from "react";
import { Separator } from "@/components/ui/separator";

const Disclaimer = () => {
  return (
    <div className="max-w-4xl mx-auto py-12 px-4">
      <h1 className="text-3xl font-bold mb-6">Disclaimer</h1>
      <p className="text-muted-foreground mb-6">Effective Date: April 28, 2025</p>
      
      <p className="mb-6">
        All services are provided "as is." We do not guarantee uptime or accuracy, 
        and we disclaim all warranties to the extent allowed by law.
      </p>
      
      <Separator className="my-10" />
      
      <p className="text-sm text-muted-foreground">
        If you have any questions about our disclaimer, please contact us at{" "}
        <a href="mailto:support@robometrics.io" className="text-primary">
          support@robometrics.io
        </a>
      </p>
    </div>
  );
};

export default Disclaimer;
