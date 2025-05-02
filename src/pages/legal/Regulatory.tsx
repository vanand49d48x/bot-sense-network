
import React, { useEffect } from "react";
import LegalLayout from "@/components/legal/LegalLayout";
import RegulatoryInformation from "@/components/legal/RegulatoryInformation";

const Regulatory = () => {
  useEffect(() => {
    // Scroll to the top of the page when component mounts
    window.scrollTo(0, 0);
  }, []);
  
  return (
    <LegalLayout>
      <RegulatoryInformation />
    </LegalLayout>
  );
};

export default Regulatory;
