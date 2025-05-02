
import React from "react";
import { Mail, MapPin } from "lucide-react";
import { Separator } from "@/components/ui/separator";

const ContactInfo = () => {
  return (
    <div className="max-w-4xl mx-auto py-12 px-4">
      <h1 className="text-3xl font-bold mb-10">Contact Information</h1>
      
      <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
        <div className="bg-card p-6 rounded-lg shadow-sm">
          <div className="flex items-center gap-3 mb-4">
            <Mail className="h-5 w-5 text-primary" />
            <h2 className="text-xl font-semibold">Email</h2>
          </div>
          <a href="mailto:support@robometrics.io" className="text-primary hover:underline">
            support@robometrics.io
          </a>
        </div>
        
        <div className="bg-card p-6 rounded-lg shadow-sm">
          <div className="flex items-center gap-3 mb-4">
            <MapPin className="h-5 w-5 text-primary" />
            <h2 className="text-xl font-semibold">Location</h2>
          </div>
          <p>Atlanta, GA, USA</p>
        </div>
      </div>
      
      <Separator className="my-10" />
      
      <h2 className="text-xl font-semibold mb-4">Business Hours</h2>
      <p className="mb-8">Monday - Friday: 9:00 AM - 5:00 PM (EST)</p>
      
      <p className="text-sm text-muted-foreground">
        Copyright: © {new Date().getFullYear()} RoboMetrics. All rights reserved.
      </p>
    </div>
  );
};

export default ContactInfo;
