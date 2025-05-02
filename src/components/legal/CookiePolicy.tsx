
import React from "react";
import { Separator } from "@/components/ui/separator";

const CookiePolicy = () => {
  return (
    <div className="max-w-4xl mx-auto py-12 px-4">
      <h1 className="text-3xl font-bold mb-6">Cookie Policy</h1>
      <p className="text-muted-foreground mb-6">Effective Date: April 28, 2025</p>
      
      <p className="mb-6">We use cookies for:</p>
      
      <ul className="list-disc pl-8 mb-6 space-y-2">
        <li>Authentication</li>
        <li>Remembering user preferences</li>
        <li>Analytics (Google Analytics, Brevo tracking)</li>
      </ul>
      
      <p className="mb-6">You can opt out by adjusting browser settings or declining cookies in the popup.</p>
      
      <Separator className="my-10" />
      
      <p className="text-sm text-muted-foreground">
        If you have any questions about our cookie policy, please contact us at{" "}
        <a href="mailto:support@robometrics.io" className="text-primary">
          support@robometrics.io
        </a>
      </p>
    </div>
  );
};

export default CookiePolicy;
