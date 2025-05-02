
import React, { useEffect } from "react";
import PlaceholderLayout from "@/components/layout/PlaceholderLayout";
import { Separator } from "@/components/ui/separator";
import { useLocation } from "react-router-dom";

const AboutContent = () => (
  <>
    <div className="prose dark:prose-invert max-w-none">
      <p className="text-lg mb-6">
        RoboMetrics is a modern monitoring platform designed to help robotics teams, developers, 
        and automation engineers keep track of their robots' health, telemetry, and performance 
        in real time. Whether you're building fleets of autonomous vehicles, managing warehouse bots, 
        or testing Arduino-based prototypes — RoboMetrics gives you full visibility into your 
        robot systems from anywhere.
      </p>
      
      <p className="text-lg mb-6">
        We believe in empowering builders with the tools they need to operate safely, efficiently, 
        and at scale. Our platform is lightweight, flexible, and built to support diverse hardware 
        and custom data formats.
      </p>
      
      <p className="text-lg mb-6">
        Our mission is to make robotics more transparent, data-driven, and reliable — one metric at a time.
      </p>
    </div>
    
    <Separator className="my-10" />
    
    <p className="text-sm text-muted-foreground">
      © {new Date().getFullYear()} RoboMetrics. All rights reserved.
    </p>
  </>
);

const About = () => {
  const location = useLocation();
  
  useEffect(() => {
    // Scroll to the top of the page when component mounts
    window.scrollTo(0, 0);
  }, []);
  
  // If accessed from dashboard (paths that start with /dashboard), 
  // render just the content without PlaceholderLayout
  if (location.pathname.startsWith("/dashboard")) {
    return (
      <div className="max-w-4xl mx-auto py-12 px-4">
        <h1 className="text-3xl font-bold mb-6">About Us</h1>
        <AboutContent />
      </div>
    );
  }
  
  // For non-dashboard access, use PlaceholderLayout
  return (
    <PlaceholderLayout title="About Us">
      <AboutContent />
    </PlaceholderLayout>
  );
};

export default About;
