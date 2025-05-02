
import React, { useEffect } from "react";
import LegalLayout from "@/components/legal/LegalLayout";
import Disclaimer from "@/components/legal/Disclaimer";

const DisclaimerPage = () => {
  useEffect(() => {
    // Scroll to the top of the page when component mounts
    window.scrollTo(0, 0);
  }, []);
  
  return (
    <LegalLayout>
      <Disclaimer />
    </LegalLayout>
  );
};

export default DisclaimerPage;
