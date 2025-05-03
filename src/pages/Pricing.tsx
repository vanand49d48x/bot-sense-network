
import React from "react";
import { MainLayout } from "@/components/layout/MainLayout";
import { Check } from "lucide-react";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Link } from "react-router-dom";

const PricingPage = () => {
  return (
    <MainLayout>
      <div className="container py-8">
        <div className="text-center mb-12">
          <h1 className="text-3xl font-bold tracking-tight">Pricing Plans</h1>
          <p className="text-muted-foreground mt-4 max-w-2xl mx-auto">
            Choose the right plan for your robotics monitoring needs. 
            All plans include our core dashboard features.
          </p>
        </div>
        
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-5 gap-6 mx-auto max-w-7xl">
          {/* Free Plan */}
          <PricingCard
            title="Free"
            price="$0"
            period="month"
            description="Hobbyists, solo developers"
            features={[
              "1 robot",
              "3 days telemetry history",
              "Live dashboard",
              "Basic API access"
            ]}
            buttonText="Get Started"
            buttonUrl="/auth"
          />
          
          {/* Starter Plan */}
          <PricingCard
            title="Starter"
            price="$19"
            period="month"
            description="Startup teams, testers"
            features={[
              "Up to 5 robots",
              "7 days history",
              "Email alerts",
              "ROS/Arduino SDK support"
            ]}
            buttonText="Subscribe"
            buttonUrl="/auth"
          />
          
          {/* Growth Plan */}
          <PricingCard
            title="Growth"
            price="$49"
            period="month"
            description="Growing teams, labs"
            features={[
              "25 robots",
              "30-day telemetry retention",
              "Custom metrics",
              "Real-time alerts",
              "Location map"
            ]}
            buttonText="Subscribe"
            buttonUrl="/auth"
            highlight={true}
          />
          
          {/* Pro Plan */}
          <PricingCard
            title="Pro"
            price="$149"
            period="month"
            description="Industrial use, fleets"
            features={[
              "100+ robots",
              "90-day history",
              "SLA + Priority support",
              "Custom alerting rules",
              "Export + team access"
            ]}
            buttonText="Subscribe"
            buttonUrl="/auth"
          />
          
          {/* Enterprise Plan */}
          <PricingCard
            title="Enterprise"
            price="Custom"
            period=""
            description="OEMs, manufacturers"
            features={[
              "Unlimited robots",
              "Custom data retention",
              "SSO / Audit logs",
              "Dedicated onboarding",
              "On-premise / MQTT support"
            ]}
            buttonText="Contact Us"
            buttonUrl="/contact"
          />
        </div>
        
        <div className="mt-16 text-center">
          <h2 className="text-2xl font-bold mb-4">Need a custom solution?</h2>
          <p className="text-muted-foreground mb-6 max-w-xl mx-auto">
            Contact our team for a tailored solution that meets your specific requirements.
          </p>
          <Link to="/contact">
            <Button variant="outline" size="lg">Contact Sales</Button>
          </Link>
        </div>
      </div>
    </MainLayout>
  );
};

// Pricing Card Component
const PricingCard = ({
  title,
  price,
  period,
  description,
  features,
  buttonText,
  buttonUrl,
  highlight = false
}: {
  title: string;
  price: string;
  period: string;
  description: string;
  features: string[];
  buttonText: string;
  buttonUrl: string;
  highlight?: boolean;
}) => {
  return (
    <Card className={`flex flex-col h-full transition-all duration-200 hover:shadow-lg ${
      highlight ? 'border-primary shadow-md' : ''
    }`}>
      <CardHeader>
        <CardTitle className="text-2xl">{title}</CardTitle>
        <div className="flex items-baseline mt-2">
          <span className="text-3xl font-bold">{price}</span>
          {period && <span className="text-muted-foreground ml-1">/{period}</span>}
        </div>
        <CardDescription>Best for {description}</CardDescription>
      </CardHeader>
      <CardContent className="flex-grow">
        <ul className="space-y-2">
          {features.map((feature, index) => (
            <li key={index} className="flex items-start">
              <Check className="h-5 w-5 text-primary shrink-0 mr-2" />
              <span>{feature}</span>
            </li>
          ))}
        </ul>
      </CardContent>
      <CardFooter>
        <Link to={buttonUrl} className="w-full">
          <Button 
            variant={highlight ? "default" : "outline"} 
            className="w-full"
          >
            {buttonText}
          </Button>
        </Link>
      </CardFooter>
    </Card>
  );
};

export default PricingPage;
