
import React, { useEffect } from "react";
import PlaceholderLayout from "@/components/layout/PlaceholderLayout";
import { Separator } from "@/components/ui/separator";

const About = () => {
  useEffect(() => {
    // Scroll to the top of the page when component mounts
    window.scrollTo(0, 0);
  }, []);
  
  return (
    <PlaceholderLayout title="About Us">
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
    </PlaceholderLayout>
  );
};

export default About;
