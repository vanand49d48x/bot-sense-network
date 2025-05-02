
import React from "react";
import { Link } from "react-router-dom";
import { MainLayout } from "./MainLayout";
import { ArrowLeft } from "lucide-react";
import { Button } from "../ui/button";

interface PlaceholderLayoutProps {
  children: React.ReactNode;
  title: string;
}

const PlaceholderLayout = ({ children, title }: PlaceholderLayoutProps) => {
  return (
    <MainLayout>
      <div className="max-w-4xl mx-auto py-12 px-4" id="page-top">
        <div className="mb-6">
          <Link to="/">
            <Button variant="ghost" size="sm" className="flex items-center gap-2">
              <ArrowLeft size={16} />
              <span>Back to Home</span>
            </Button>
          </Link>
        </div>
        <h1 className="text-3xl font-bold mb-6">{title}</h1>
        {children}
      </div>
    </MainLayout>
  );
};

export default PlaceholderLayout;
