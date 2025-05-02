
import React from "react";
import { Button } from "@/components/ui/button";
import { Link } from "react-router-dom";
import { Battery, AlertTriangle, MapPin, Shield, Activity, 
         Bot, Rocket, Clock, Mail, Map, Bell } from "lucide-react";

const Landing = () => {
  return (
    <div className="min-h-screen bg-background flex flex-col">
      {/* Header/Navigation */}
      <header className="border-b border-border/40 p-4">
        <div className="container mx-auto flex justify-between items-center">
          <div className="flex items-center gap-2">
            <Bot className="h-6 w-6 text-primary" />
            <span className="text-xl font-bold">RoboMetrics</span>
          </div>
          <div className="flex gap-4">
            <Link to="/auth" className="text-muted-foreground hover:text-foreground transition-colors">
              Login
            </Link>
            <Link to="/auth" className="text-muted-foreground hover:text-foreground transition-colors">
              Sign Up
            </Link>
          </div>
        </div>
      </header>

      {/* Hero Section */}
      <section className="py-16 md:py-24 container mx-auto px-4">
        <div className="max-w-4xl mx-auto text-center">
          <h1 className="text-4xl md:text-5xl lg:text-6xl font-bold mb-6">
            RoboMetrics — Monitor Your Robots. Anytime. Anywhere.
          </h1>
          <p className="text-xl text-muted-foreground mb-8 max-w-2xl mx-auto">
            Track your robots' health, battery, temperature, and location in real-time.
            Stay ahead of failures, maximize uptime, and manage fleets smarter.
          </p>
          <div className="flex flex-col sm:flex-row gap-4 justify-center">
            <Link to="/auth">
              <Button size="lg" className="px-8 py-6 text-lg">
                Start Free Trial
              </Button>
            </Link>
            <Link to="/integration">
              <Button size="lg" variant="outline" className="px-8 py-6 text-lg">
                Learn More
              </Button>
            </Link>
          </div>
        </div>
      </section>

      {/* Why Section */}
      <section className="py-16 bg-muted">
        <div className="container mx-auto px-4">
          <h2 className="text-3xl md:text-4xl font-bold mb-12 text-center">⚡ Why RoboMetrics?</h2>
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-8 max-w-4xl mx-auto">
            <FeatureCard 
              icon={<Activity />}
              title="Real-Time Robot Telemetry Dashboard"
            />
            <FeatureCard 
              icon={<Battery />}
              title="Battery Health Alerts & Predictive Maintenance"
            />
            <FeatureCard 
              icon={<Rocket />}
              title="Easy Robot API Integration"
              subtitle="Arduino, ESP32, Raspberry Pi, ROS"
            />
            <div className="lg:col-span-3 md:col-span-2 grid grid-cols-2 gap-8 w-full">
              <FeatureCard 
                icon={<MapPin />}
                title="Live Map Tracking for Mobile Robots"
              />
              <FeatureCard 
                icon={<Bell />}
                title="Smart Notifications for Critical Events"
              />
            </div>
          </div>
        </div>
      </section>

      {/* Features Section - Center justified */}
      <section className="py-16 container mx-auto px-4">
        <h2 className="text-3xl md:text-4xl font-bold mb-10 text-center">🔥 Features</h2>
        <div className="grid md:grid-cols-2 gap-16 mt-12 max-w-3xl mx-auto">
          {/* Left Column */}
          <div className="bg-card rounded-lg p-6 shadow-sm flex flex-col">
            <h3 className="text-xl font-semibold mb-6 text-center">Telemetry Monitoring</h3>
            <ul className="space-y-4 flex-grow">
              <li className="flex items-center gap-4">
                <div className="bg-primary/10 p-2 rounded-md">
                  <Battery className="text-primary h-5 w-5" />
                </div>
                <span>Battery %</span>
              </li>
              <li className="flex items-center gap-4">
                <div className="bg-primary/10 p-2 rounded-md">
                  <Activity className="text-primary h-5 w-5" />
                </div>
                <span>Status</span>
              </li>
              <li className="flex items-center gap-4">
                <div className="bg-primary/10 p-2 rounded-md">
                  <MapPin className="text-primary h-5 w-5" />
                </div>
                <span>Location</span>
              </li>
              <li className="flex items-center gap-4">
                <div className="bg-primary/10 p-2 rounded-md">
                  <AlertTriangle className="text-primary h-5 w-5" />
                </div>
                <span>Health</span>
              </li>
            </ul>
          </div>
          
          {/* Right Column */}
          <div className="bg-card rounded-lg p-6 shadow-sm flex flex-col">
            <div className="space-y-8 flex-grow">
              <div>
                <h3 className="text-xl font-semibold mb-4 flex items-center gap-2">
                  <div className="bg-primary/10 p-2 rounded-md">
                    <Bell className="text-primary h-5 w-5" />
                  </div>
                  Custom Alerts
                </h3>
                <p className="text-muted-foreground">
                  Get email or app alerts for low battery or sensor warnings.
                </p>
              </div>
              
              <div>
                <h3 className="text-xl font-semibold mb-4 flex items-center gap-2">
                  <div className="bg-primary/10 p-2 rounded-md">
                    <Bot className="text-primary h-5 w-5" />
                  </div>
                  Fleet Management
                </h3>
                <p className="text-muted-foreground">
                  Monitor hundreds of robots in one unified dashboard.
                </p>
              </div>
              
              <div>
                <h3 className="text-xl font-semibold mb-4 flex items-center gap-2">
                  <div className="bg-primary/10 p-2 rounded-md">
                    <Rocket className="text-primary h-5 w-5" />
                  </div>
                  Open APIs
                </h3>
                <p className="text-muted-foreground">
                  Easy integration with any robot type.
                </p>
              </div>
            </div>
          </div>
        </div>
      </section>

      {/* How it Works */}
      <section className="py-16 bg-muted">
        <div className="container mx-auto px-4">
          <h2 className="text-3xl md:text-4xl font-bold mb-12 text-center">🛠 How It Works</h2>
          <div className="grid grid-cols-1 md:grid-cols-3 gap-8 max-w-4xl mx-auto">
            <div className="bg-card p-8 rounded-lg shadow-sm flex flex-col items-center text-center">
              <div className="bg-primary/10 p-4 rounded-full mb-4">
                <Bot className="h-8 w-8 text-primary" />
              </div>
              <h3 className="text-xl font-semibold mb-2">Register your robot</h3>
              <p className="text-muted-foreground">Create an account and add your robot to the dashboard</p>
            </div>
            
            <div className="bg-card p-8 rounded-lg shadow-sm flex flex-col items-center text-center">
              <div className="bg-primary/10 p-4 rounded-full mb-4">
                <Rocket className="h-8 w-8 text-primary" />
              </div>
              <h3 className="text-xl font-semibold mb-2">Add 10 lines of code</h3>
              <p className="text-muted-foreground">Simple integration with your robot's software</p>
            </div>
            
            <div className="bg-card p-8 rounded-lg shadow-sm flex flex-col items-center text-center">
              <div className="bg-primary/10 p-4 rounded-full mb-4">
                <Bell className="h-8 w-8 text-primary" />
              </div>
              <h3 className="text-xl font-semibold mb-2">Monitor and receive alerts</h3>
              <p className="text-muted-foreground">Get instant notifications about your robot's status</p>
            </div>
          </div>
        </div>
      </section>

      {/* Perfect For */}
      <section className="py-16 container mx-auto px-4">
        <h2 className="text-3xl md:text-4xl font-bold mb-12 text-center">✨ Perfect For</h2>
        <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-5 gap-6 max-w-5xl mx-auto">
          <PerfectForCard title="Robotics Startups" />
          <PerfectForCard title="Automation Engineers" />
          <PerfectForCard title="Drone Operators" />
          <PerfectForCard title="Mobile Robot Fleet Managers" />
          <PerfectForCard title="Educational Robotics Labs" />
        </div>
      </section>

      {/* CTA */}
      <section className="py-16 bg-primary/5">
        <div className="container mx-auto px-4 text-center">
          <h2 className="text-3xl md:text-4xl font-bold mb-4">📈 Get Started Today</h2>
          <p className="text-xl mb-8 max-w-2xl mx-auto">
            Sign up free and start monitoring your first robot in minutes.
          </p>
          <div className="flex flex-col sm:flex-row gap-4 justify-center">
            <Link to="/auth">
              <Button size="lg" className="px-8 py-6 text-lg">
                Start Free Trial
              </Button>
            </Link>
            <Link to="/integration">
              <Button size="lg" variant="outline" className="px-8 py-6 text-lg">
                Learn More
              </Button>
            </Link>
          </div>
        </div>
      </section>

      {/* Trust Section */}
      <section className="py-16 container mx-auto px-4">
        <h2 className="text-3xl md:text-4xl font-bold mb-12 text-center">🛡️ Trusted. Secure. Fast.</h2>
        <div className="grid grid-cols-1 md:grid-cols-3 gap-8 max-w-4xl mx-auto">
          <div className="flex flex-col items-center text-center">
            <Shield className="h-10 w-10 text-primary mb-4" />
            <h3 className="text-xl font-semibold mb-2">Bank-grade SSL Encryption</h3>
          </div>
          <div className="flex flex-col items-center text-center">
            <Shield className="h-10 w-10 text-primary mb-4" />
            <h3 className="text-xl font-semibold mb-2">Data Privacy First</h3>
          </div>
          <div className="flex flex-col items-center text-center">
            <Shield className="h-10 w-10 text-primary mb-4" />
            <h3 className="text-xl font-semibold mb-2">Designed for reliability and uptime</h3>
          </div>
        </div>
      </section>

      {/* Footer */}
      <footer className="py-8 border-t border-border">
        <div className="container mx-auto px-4">
          <div className="flex justify-between items-center">
            <div className="flex items-center gap-2">
              <Bot className="h-5 w-5 text-primary" />
              <span className="font-semibold">RoboMetrics</span>
            </div>
            <p className="text-sm text-muted-foreground">
              © {new Date().getFullYear()} RoboMetrics. All rights reserved.
            </p>
          </div>
        </div>
      </footer>
    </div>
  );
};

const FeatureCard = ({ icon, title, subtitle }: { 
  icon: React.ReactNode;
  title: string;
  subtitle?: string;
}) => {
  return (
    <div className="bg-card p-6 rounded-lg shadow-sm">
      <div className="flex items-start gap-4">
        <div className="bg-primary/10 p-2 rounded-md">
          {icon}
        </div>
        <div>
          <h3 className="font-semibold mb-1">{title}</h3>
          {subtitle && <p className="text-sm text-muted-foreground">{subtitle}</p>}
        </div>
      </div>
    </div>
  );
};

const PerfectForCard = ({ title }: { title: string }) => {
  return (
    <div className="bg-card p-4 rounded-lg shadow-sm flex justify-center items-center text-center">
      <span className="font-medium">{title}</span>
    </div>
  );
};

export default Landing;
