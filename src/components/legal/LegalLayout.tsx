
import React, { useEffect } from "react";
import { MainLayout } from "../layout/MainLayout";

interface LegalLayoutProps {
  children: React.ReactNode;
}

const LegalLayout = ({ children }: LegalLayoutProps) => {
  useEffect(() => {
    // Scroll to the top of the page when component mounts
    window.scrollTo(0, 0);
  }, []);

  return (
    <MainLayout>
      <div className="bg-background min-h-screen" id="page-top">
        {children}
      </div>
    </MainLayout>
  );
};

export default LegalLayout;
