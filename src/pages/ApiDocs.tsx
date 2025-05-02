
import React, { useEffect } from "react";
import PlaceholderLayout from "@/components/layout/PlaceholderLayout";
import { Code } from "@/components/ui/code";

const ApiDocs = () => {
  useEffect(() => {
    // Scroll to the top of the page when component mounts
    window.scrollTo(0, 0);
  }, []);
  
  return (
    <PlaceholderLayout title="API Documentation">
      <p className="text-muted-foreground mb-6">
        This page will contain detailed API documentation. Coming soon!
      </p>
      
      <div className="mb-8">
        <h2 className="text-xl font-semibold mb-4">Quick Start Example</h2>
        <Code>
          {`
// Example API call
const response = await fetch('https://api.robometrics.io/v1/robots', {
  headers: {
    'Authorization': 'Bearer YOUR_API_KEY'
  }
});

const robots = await response.json();
          `}
        </Code>
      </div>
      
      <p className="text-center text-muted-foreground">
        Full documentation coming soon!
      </p>
    </PlaceholderLayout>
  );
};

export default ApiDocs;
