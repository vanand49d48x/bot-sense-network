
import React, { useEffect } from "react";
import LegalLayout from "@/components/legal/LegalLayout";
import PrivacyPolicy from "@/components/legal/PrivacyPolicy";

const Privacy = () => {
  useEffect(() => {
    // Scroll to the top of the page when component mounts
    window.scrollTo(0, 0);
  }, []);
  
  return (
    <LegalLayout>
      <PrivacyPolicy />
    </LegalLayout>
  );
};

export default Privacy;
