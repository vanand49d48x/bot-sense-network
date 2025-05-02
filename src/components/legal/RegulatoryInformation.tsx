
import React from "react";
import { Separator } from "@/components/ui/separator";

const RegulatoryInformation = () => {
  return (
    <div className="max-w-4xl mx-auto py-12 px-4">
      <h1 className="text-3xl font-bold mb-6">Regulatory Information</h1>
      <p className="text-muted-foreground mb-6">Effective Date: April 28, 2025</p>
      
      <p className="mb-6">RoboMetrics complies with the following regulations and standards where applicable:</p>
      
      <div className="space-y-6">
        <div>
          <h2 className="text-xl font-semibold mb-2">General Data Protection Regulation (GDPR)</h2>
          <p>We handle personal data in accordance with GDPR principles, ensuring transparency, security, and user control.</p>
        </div>
        
        <div>
          <h2 className="text-xl font-semibold mb-2">California Consumer Privacy Act (CCPA)</h2>
          <p>We provide California residents with clear data rights, including the ability to request, delete, or restrict access to personal data.</p>
        </div>
        
        <div>
          <h2 className="text-xl font-semibold mb-2">CAN-SPAM Act</h2>
          <p>We adhere to email best practices, offering users the ability to opt out of communications and honoring all unsubscribe requests.</p>
        </div>
      </div>
      
      <div className="mt-8 p-4 bg-muted/50 rounded-lg">
        <p className="mb-0">
          While we are not currently HIPAA or SOC 2 certified, we take data security and integrity seriously. For enterprise-grade needs or compliance partnerships, please contact <a href="mailto:support@robometrics.io" className="text-primary font-medium">support@robometrics.io</a>.
        </p>
      </div>
      
      <Separator className="my-10" />
      
      <p className="text-sm text-muted-foreground">
        If you have any questions about our regulatory information, please contact us at{" "}
        <a href="mailto:support@robometrics.io" className="text-primary">
          support@robometrics.io
        </a>
      </p>
    </div>
  );
};

export default RegulatoryInformation;
