
import React, { useEffect } from "react";
import LegalLayout from "@/components/legal/LegalLayout";
import ContactInfo from "@/components/legal/ContactInfo";

const Contact = () => {
  useEffect(() => {
    // Scroll to the top of the page when component mounts
    window.scrollTo(0, 0);
  }, []);
  
  return (
    <LegalLayout>
      <ContactInfo />
    </LegalLayout>
  );
};

export default Contact;
