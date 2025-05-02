
import React from "react";
import { MainLayout } from "../layout/MainLayout";

interface LegalLayoutProps {
  children: React.ReactNode;
}

const LegalLayout = ({ children }: LegalLayoutProps) => {
  return (
    <MainLayout>
      <div className="bg-background min-h-screen">
        {children}
      </div>
    </MainLayout>
  );
};

export default LegalLayout;
