
import React, { useEffect } from "react";
import { Check } from "lucide-react";
import PlaceholderLayout from "@/components/layout/PlaceholderLayout";
import { Button } from "@/components/ui/button";
import { Link } from "react-router-dom";
import { useCart } from "@/context/CartContext";
import { toast } from "@/components/ui/sonner";
import { useAuth } from "@/context/AuthContext";

const tiers = [
  {
    id: "tier_free",
    name: "Free",
    price: 0,
    period: "month",
    description: "For hobbyists and solo developers",
    features: [
      "1 robot",
      "3 days telemetry history",
      "Live dashboard",
      "Basic API access",
    ],
    cta: "Start for Free",
    link: "/auth",
    highlight: false,
  },
  {
    id: "tier_starter",
    name: "Starter",
    price: 1900,
    period: "month",
    description: "For startup teams and testers",
    features: [
      "Up to 5 robots",
      "7 days history",
      "Email alerts",
      "ROS/Arduino SDK support",
    ],
    cta: "Get Started",
    link: "/auth",
    highlight: false,
  },
  {
    id: "tier_growth",
    name: "Growth",
    price: 4900,
    period: "month",
    description: "For growing teams and labs",
    features: [
      "25 robots",
      "30-day telemetry retention",
      "Custom metrics",
      "Real-time alerts",
      "Location map",
    ],
    cta: "Choose Growth",
    link: "/auth",
    highlight: true,
  },
  {
    id: "tier_pro",
    name: "Pro",
    price: 14900,
    period: "month",
    description: "For industrial use and fleets",
    features: [
      "100+ robots",
      "90-day history",
      "SLA + Priority support",
      "Custom alerting rules",
      "Export + team access",
    ],
    cta: "Choose Pro",
    link: "/auth",
    highlight: false,
  },
  {
    id: "tier_enterprise",
    name: "Enterprise",
    price: 0, // Custom pricing
    period: "",
    description: "For OEMs and manufacturers",
    features: [
      "Unlimited robots",
      "Custom data retention",
      "SSO / Audit logs",
      "Dedicated onboarding",
      "On-premise / MQTT support",
    ],
    cta: "Contact Sales",
    link: "/contact",
    highlight: false,
  },
];

const addons = [
  {
    id: "addon_storage",
    name: "Additional telemetry storage",
    price: 500,
    description: "Per extra 30 days",
    type: 'addon'
  },
  {
    id: "addon_robot",
    name: "Extra robot",
    price: 200,
    description: "Per bot on Free/Starter plans",
    type: 'addon'
  },
  {
    id: "addon_seats",
    name: "Team seats",
    price: 1000,
    description: "Per extra admin",
    type: 'addon'
  }
];

const PricingTier = ({ tier }: { tier: typeof tiers[0] }) => {
  const { addToCart, isInCart } = useCart();
  const { user } = useAuth();
  
  const handleAddToCart = () => {
    if (tier.price === 0 && tier.name === "Enterprise") {
      // For Enterprise, redirect to contact page
      return;
    }
    
    addToCart({
      id: tier.id,
      name: `${tier.name} Plan`,
      price: tier.price,
      type: 'subscription',
      period: tier.period
    });
  };
  
  const inCart = isInCart(tier.id);

  return (
    <div className={`flex flex-col p-6 rounded-xl border transition-all duration-300 
      ${tier.highlight ? 'border-primary shadow-md relative' : 'border-border'} 
      hover:border-primary hover:shadow-md`}>
      {tier.highlight && (
        <span className="absolute -top-3 left-1/2 transform -translate-x-1/2 bg-primary text-primary-foreground text-xs font-medium px-3 py-1 rounded-full">
          MOST POPULAR
        </span>
      )}
      
      <div className="mb-5">
        <h3 className="text-xl font-semibold">{tier.name}</h3>
        <div className="flex items-baseline mt-2">
          <span className="text-3xl font-bold">${(tier.price/100).toFixed(2)}</span>
          {tier.period && <span className="text-muted-foreground ml-1">/{tier.period}</span>}
        </div>
        <p className="text-sm text-muted-foreground mt-2">{tier.description}</p>
      </div>
      
      <ul className="space-y-3 mb-8 flex-grow">
        {tier.features.map((feature, index) => (
          <li key={index} className="flex items-start">
            <Check className="h-5 w-5 text-primary shrink-0 mr-2" />
            <span className="text-sm">{feature}</span>
          </li>
        ))}
      </ul>
      
      {tier.name === "Enterprise" ? (
        <Link to={tier.link} className="mt-auto">
          <Button 
            variant="outline" 
            className="w-full hover:bg-primary hover:text-primary-foreground transition-colors"
          >
            {tier.cta}
          </Button>
        </Link>
      ) : tier.name === "Free" ? (
        <Link to={user ? "/dashboard" : "/auth"} className="mt-auto">
          <Button 
            variant="outline" 
            className="w-full hover:bg-primary hover:text-primary-foreground transition-colors"
          >
            {user ? "Access Dashboard" : "Start for Free"}
          </Button>
        </Link>
      ) : (
        <Button 
          variant={tier.highlight ? "default" : "outline"}
          className="w-full hover:bg-primary hover:text-primary-foreground transition-colors mt-auto"
          onClick={handleAddToCart}
          disabled={inCart}
        >
          {inCart ? "Added to Cart" : tier.cta}
        </Button>
      )}
    </div>
  );
};

const AddonItem = ({ addon }: { addon: typeof addons[0] }) => {
  const { addToCart, isInCart } = useCart();
  
  const handleAddToCart = () => {
    addToCart({
      id: addon.id,
      name: addon.name,
      price: addon.price,
      type: 'addon',
      quantity: 1
    });
  };
  
  const inCart = isInCart(addon.id);

  return (
    <div className="bg-card p-6 rounded-lg border border-border hover:border-primary hover:shadow-md transition-all duration-300">
      <h4 className="font-medium mb-2">{addon.name}</h4>
      <p className="text-lg font-bold">${(addon.price/100).toFixed(2)}/month</p>
      <p className="text-sm text-muted-foreground mb-4">{addon.description}</p>
      <Button 
        variant="outline" 
        size="sm" 
        className="w-full"
        onClick={handleAddToCart}
        disabled={inCart}
      >
        {inCart ? "Added to Cart" : "Add to Cart"}
      </Button>
    </div>
  );
};

const Pricing = () => {
  useEffect(() => {
    // Scroll to the top of the page when component mounts
    window.scrollTo(0, 0);
  }, []);
  
  return (
    <PlaceholderLayout title="Pricing">
      <div className="max-w-6xl mx-auto">
        <div className="text-center mb-12">
          <h2 className="text-3xl font-bold mb-3">Start Free. Scale as You Grow.</h2>
          <p className="text-lg text-muted-foreground max-w-2xl mx-auto">
            Monitor your first robot for free. No credit card required.
            Add more robots and features as your fleet grows.
          </p>
        </div>
        
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 xl:grid-cols-5 gap-6 mb-16">
          {tiers.map((tier, index) => (
            <PricingTier key={index} tier={tier} />
          ))}
        </div>
        
        <div className="mt-16">
          <h3 className="text-xl font-semibold mb-6 text-center">Add-ons (Optional per-tier or à la carte)</h3>
          <div className="grid grid-cols-1 md:grid-cols-3 gap-6 max-w-3xl mx-auto">
            {addons.map((addon, index) => (
              <AddonItem key={index} addon={addon} />
            ))}
          </div>
        </div>
        
        <div className="text-center mt-16">
          <h3 className="text-xl font-semibold mb-4">Have questions about our pricing?</h3>
          <p className="mb-6 text-muted-foreground">
            Our team is ready to help you find the right plan for your needs.
          </p>
          <Link to="/contact">
            <Button size="lg">Contact Sales</Button>
          </Link>
        </div>
      </div>
    </PlaceholderLayout>
  );
};

export default Pricing;
