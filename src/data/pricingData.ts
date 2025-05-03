
// Pricing tiers data
export const tiers = [
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
    cta: "Subscribe Now",
    link: "/checkout?plan=starter",
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
    cta: "Subscribe Now",
    link: "/checkout?plan=growth",
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
    cta: "Subscribe Now",
    link: "/checkout?plan=pro",
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

// Add-ons data
export const addons = [
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
