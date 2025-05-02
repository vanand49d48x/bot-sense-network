
import React from "react";
import { Separator } from "@/components/ui/separator";

const TermsOfService = () => {
  return (
    <div className="max-w-4xl mx-auto py-12 px-4">
      <h1 className="text-3xl font-bold mb-6">Terms of Service</h1>
      <p className="text-muted-foreground mb-6">Effective Date: April 28, 2025</p>
      
      <p className="mb-6">By using RoboMetrics, you agree to the following terms:</p>
      
      <ul className="list-disc pl-8 mb-6 space-y-2">
        <li>You are responsible for the data sent from your robots.</li>
        <li>RoboMetrics is not liable for hardware or operational failure.</li>
        <li>You may not misuse the API or service to overload our systems.</li>
        <li>We reserve the right to suspend access for abuse.</li>
      </ul>
      
      <p className="mb-6">Use of RoboMetrics constitutes acceptance of these terms. Full terms at <a href="https://robometrics.xyz/terms" className="text-primary underline">robometrics.xyz/terms</a></p>
      
      <Separator className="my-10" />
      
      <p className="text-sm text-muted-foreground">
        If you have any questions about our terms of service, please contact us at{" "}
        <a href="mailto:support@robometrics.io" className="text-primary">
          support@robometrics.io
        </a>
      </p>
    </div>
  );
};

export default TermsOfService;
