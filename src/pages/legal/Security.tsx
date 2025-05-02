
import React, { useEffect } from "react";
import LegalLayout from "@/components/legal/LegalLayout";
import SecurityMeasures from "@/components/legal/SecurityMeasures";

const Security = () => {
  useEffect(() => {
    // Scroll to the top of the page when component mounts
    window.scrollTo(0, 0);
  }, []);
  
  return (
    <LegalLayout>
      <SecurityMeasures />
    </LegalLayout>
  );
};

export default Security;
