
import React from "react";
import { Separator } from "@/components/ui/separator";

const RegulatoryInformation = () => {
  return (
    <div className="max-w-4xl mx-auto py-12 px-4">
      <h1 className="text-3xl font-bold mb-6">Regulatory Information</h1>
      <p className="text-muted-foreground mb-6">Effective Date: April 28, 2025</p>
      
      <p className="mb-6">RoboMetrics complies with:</p>
      
      <ul className="list-disc pl-8 mb-6 space-y-2">
        <li>General Data Protection Regulation (GDPR)</li>
        <li>California Consumer Privacy Act (CCPA)</li>
        <li>CAN-SPAM for email</li>
      </ul>
      
      <p className="mb-6">
        We are not yet HIPAA or SOC2 certified. Please contact support for enterprise agreements.
      </p>
      
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
