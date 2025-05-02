
import React, { useEffect } from "react";
import PlaceholderLayout from "@/components/layout/PlaceholderLayout";
import { Card, CardHeader, CardTitle, CardDescription, CardContent, CardFooter } from "@/components/ui/card";
import { Button } from "@/components/ui/button";

const Blog = () => {
  useEffect(() => {
    // Scroll to the top of the page when component mounts
    window.scrollTo(0, 0);
  }, []);
  
  return (
    <PlaceholderLayout title="Blog">
      <p className="text-muted-foreground mb-8">
        Stay updated with the latest news, tutorials, and insights about RoboMetrics and robotics monitoring.
      </p>
      
      <div className="text-center py-12">
        <h2 className="text-2xl font-bold mb-4">Coming Soon!</h2>
        <p className="text-muted-foreground mb-6">
          Our blog is currently under development. Check back soon for articles, tutorials, and updates!
        </p>
        <Button>Subscribe to Updates</Button>
      </div>
    </PlaceholderLayout>
  );
};

export default Blog;
