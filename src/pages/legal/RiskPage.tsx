
import React, { useEffect } from "react";
import LegalLayout from "@/components/legal/LegalLayout";
import RiskDisclosure from "@/components/legal/RiskDisclosure";

const RiskPage = () => {
  useEffect(() => {
    // Scroll to the top of the page when component mounts
    window.scrollTo(0, 0);
  }, []);
  
  return (
    <LegalLayout>
      <RiskDisclosure />
    </LegalLayout>
  );
};

export default RiskPage;
