
import React, { useState } from "react";
import { Input } from "@/components/ui/input";
import { Button } from "@/components/ui/button";
import { toast } from "@/components/ui/sonner";

const NewsletterForm = () => {
  const [email, setEmail] = useState("");
  const [isSubmitting, setIsSubmitting] = useState(false);
  
  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    // Basic email validation
    if (!email || !/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email)) {
      toast.error("Please enter a valid email address");
      return;
    }
    
    setIsSubmitting(true);
    
    try {
      // For now, just simulate a successful subscription
      // In a real implementation, you would send this to your backend or email service
      console.log("Newsletter subscription for:", email);
      
      // Simulate API call
      await new Promise(resolve => setTimeout(resolve, 800));
      
      toast.success("Thank you for subscribing to our newsletter!");
      setEmail("");
    } catch (error) {
      console.error("Newsletter subscription failed:", error);
      toast.error("Subscription failed. Please try again later.");
    } finally {
      setIsSubmitting(false);
    }
  };
  
  return (
    <form onSubmit={handleSubmit} className="flex gap-2">
      <Input 
        type="email" 
        placeholder="Your email"
        value={email}
        onChange={(e) => setEmail(e.target.value)}
        className="bg-background text-sm px-3 py-2 rounded border border-border flex-1"
        aria-label="Email for newsletter"
        disabled={isSubmitting}
      />
      <Button size="sm" type="submit" disabled={isSubmitting}>
        {isSubmitting ? "Subscribing..." : "Subscribe"}
      </Button>
    </form>
  );
};

export default NewsletterForm;
