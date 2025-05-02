
import React from "react";
import { Separator } from "@/components/ui/separator";
import DataRequestForm from "./DataRequestForm";

const PrivacyPolicy = () => {
  return (
    <div className="max-w-4xl mx-auto py-12 px-4">
      <h1 className="text-3xl font-bold mb-6">Privacy Policy</h1>
      <p className="text-muted-foreground mb-6">Effective Date: April 28, 2025</p>
      
      <p className="mb-6">RoboMetrics respects your privacy. This Privacy Policy explains how we collect, use, and protect your information.</p>
      
      <h2 className="text-xl font-semibold mt-8 mb-4">What We Collect</h2>
      <ul className="list-disc pl-8 mb-6 space-y-2">
        <li>Telemetry data sent from robots</li>
        <li>User-provided emails, names, and login info</li>
        <li>Device metadata (IP, browser, etc.)</li>
      </ul>
      
      <h2 className="text-xl font-semibold mt-8 mb-4">Why We Collect It</h2>
      <ul className="list-disc pl-8 mb-6 space-y-2">
        <li>To display metrics dashboards and improve our services</li>
        <li>To communicate updates and support</li>
        <li>For analytics and platform diagnostics</li>
      </ul>
      
      <h2 className="text-xl font-semibold mt-8 mb-4">Your Rights</h2>
      <ul className="list-disc pl-8 mb-6 space-y-2">
        <li>Request access or deletion of your data</li>
        <li>Opt out of communications</li>
      </ul>
      
      <p className="mt-6 mb-8">We never sell your data. Full details at <a href="https://robometrics.io/privacy" className="text-primary underline">robometrics.io/privacy</a></p>
      
      <h2 className="text-xl font-semibold mt-8 mb-4">Exercise Your Data Rights</h2>
      <p className="mb-6">
        Under various privacy regulations including GDPR and CCPA, you have the right to access, 
        correct, or delete your personal data. Use the form below to submit your request, 
        or email us directly at <a href="mailto:support@robometrics.io" className="text-primary">
        support@robometrics.io</a>.
      </p>
      
      <DataRequestForm />
      
      <Separator className="my-10" />
      
      <p className="text-sm text-muted-foreground">
        If you have any questions about our privacy policy, please contact us at{" "}
        <a href="mailto:support@robometrics.io" className="text-primary">
          support@robometrics.io
        </a>
      </p>
    </div>
  );
};

export default PrivacyPolicy;
