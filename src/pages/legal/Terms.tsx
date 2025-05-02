
import React, { useEffect } from "react";
import LegalLayout from "@/components/legal/LegalLayout";
import TermsOfService from "@/components/legal/TermsOfService";

const Terms = () => {
  useEffect(() => {
    // Scroll to the top of the page when component mounts
    window.scrollTo(0, 0);
  }, []);
  
  return (
    <LegalLayout>
      <TermsOfService />
    </LegalLayout>
  );
};

export default Terms;
