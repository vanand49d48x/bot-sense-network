
import React, { useEffect } from "react";
import LegalLayout from "@/components/legal/LegalLayout";
import CookiePolicy from "@/components/legal/CookiePolicy";

const Cookies = () => {
  useEffect(() => {
    // Scroll to the top of the page when component mounts
    window.scrollTo(0, 0);
  }, []);
  
  return (
    <LegalLayout>
      <CookiePolicy />
    </LegalLayout>
  );
};

export default Cookies;
