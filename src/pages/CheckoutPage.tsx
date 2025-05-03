
import React, { useEffect, useState } from 'react';
import PlaceholderLayout from "@/components/layout/PlaceholderLayout";
import { Button } from "@/components/ui/button";
import { Card, CardContent } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import { Check, ArrowLeft } from "lucide-react";
import { useAuth } from "@/context/AuthContext";
import { supabase } from "@/integrations/supabase/client";
import { toast } from "@/components/ui/sonner";
import { useNavigate, useLocation, Link } from "react-router-dom";
import { tiers, addons } from "@/data/pricingData";

const CheckoutPage = () => {
  const { user } = useAuth();
  const navigate = useNavigate();
  const location = useLocation();
  const [isLoading, setIsLoading] = useState(false);
  const [selectedPlan, setSelectedPlan] = useState<typeof tiers[0] | null>(null);
  const [selectedAddons, setSelectedAddons] = useState<(typeof addons[0] & { quantity: number })[]>([]);
  
  // Get plan and addon from URL params
  useEffect(() => {
    const params = new URLSearchParams(location.search);
    const planId = params.get('plan');
    const addonId = params.get('addon');
    
    if (planId) {
      const foundPlan = tiers.find(t => t.id === `tier_${planId}` || t.id === planId);
      if (foundPlan && foundPlan.price > 0) {
        setSelectedPlan(foundPlan);
      }
    }
    
    if (addonId) {
      const foundAddon = addons.find(a => a.id === `addon_${addonId}` || a.id === addonId);
      if (foundAddon) {
        setSelectedAddons(prev => {
          const existing = prev.find(a => a.id === foundAddon.id);
          if (existing) {
            return prev.map(a => a.id === foundAddon.id ? { ...a, quantity: a.quantity + 1 } : a);
          } else {
            return [...prev, { ...foundAddon, quantity: 1 }];
          }
        });
      }
    }
  }, [location.search]);
  
  // Redirect to login if not logged in
  useEffect(() => {
    if (!user) {
      toast.error("Please log in to proceed with checkout");
      navigate("/auth", { state: { redirectUrl: "/checkout" } });
    }
  }, [user, navigate]);
  
  // Calculate totals
  const planPrice = selectedPlan?.price || 0;
  const addonTotal = selectedAddons.reduce((sum, addon) => sum + (addon.price * addon.quantity), 0);
  const total = planPrice + addonTotal;
  
  const handleQuantityChange = (addonId: string, change: number) => {
    setSelectedAddons(prev => {
      return prev.map(addon => {
        if (addon.id === addonId) {
          const newQuantity = Math.max(1, addon.quantity + change);
          return { ...addon, quantity: newQuantity };
        }
        return addon;
      });
    });
  };
  
  const handleRemoveAddon = (addonId: string) => {
    setSelectedAddons(prev => prev.filter(addon => addon.id !== addonId));
  };
  
  const handleAddAddon = (addon: typeof addons[0]) => {
    setSelectedAddons(prev => {
      const existing = prev.find(a => a.id === addon.id);
      if (existing) {
        return prev.map(a => a.id === addon.id ? { ...a, quantity: a.quantity + 1 } : a);
      } else {
        return [...prev, { ...addon, quantity: 1 }];
      }
    });
  };
  
  const handleCheckout = async () => {
    if (!user) {
      toast.error("Please log in to proceed with checkout");
      return;
    }
    
    if (!selectedPlan && selectedAddons.length === 0) {
      toast.error("Please select a plan or add-on");
      return;
    }
    
    setIsLoading(true);
    
    try {
      // Prepare items for checkout
      const items = [];
      
      // Add plan if selected
      if (selectedPlan) {
        items.push({
          id: selectedPlan.id,
          name: `${selectedPlan.name} Plan`,
          price: selectedPlan.price,
          type: 'subscription',
          period: selectedPlan.period
        });
      }
      
      // Add add-ons
      selectedAddons.forEach(addon => {
        items.push({
          id: addon.id,
          name: addon.name,
          price: addon.price,
          type: 'addon',
          quantity: addon.quantity
        });
      });
      
      const { data, error } = await supabase.functions.invoke('create-checkout', {
        body: {
          items,
          redirectUrl: window.location.origin
        }
      });
      
      if (error) throw error;
      
      if (data?.url) {
        window.location.href = data.url;
      } else {
        throw new Error("No checkout URL returned");
      }
    } catch (error) {
      console.error("Checkout error:", error);
      toast.error("Failed to create checkout session. Please try again.");
    } finally {
      setIsLoading(false);
    }
  };
  
  return (
    <PlaceholderLayout title="Checkout">
      <div className="max-w-5xl mx-auto px-4">
        <div className="mb-8">
          <Link to="/pricing">
            <Button variant="ghost" size="sm" className="flex items-center gap-2">
              <ArrowLeft size={16} />
              <span>Back to Pricing</span>
            </Button>
          </Link>
        </div>
        
        <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
          {/* Selected Plan and Add-ons */}
          <div className="md:col-span-2 space-y-6">
            {selectedPlan ? (
              <Card>
                <CardContent className="pt-6">
                  <h2 className="text-lg font-medium mb-4">Selected Plan</h2>
                  <div className="flex items-center justify-between py-4 border-b">
                    <div>
                      <h3 className="font-medium">{selectedPlan.name} Plan</h3>
                      <p className="text-sm text-muted-foreground">
                        ${(selectedPlan.price/100).toFixed(2)}/{selectedPlan.period}
                      </p>
                      <ul className="mt-2 space-y-1">
                        {selectedPlan.features.slice(0, 3).map((feature, i) => (
                          <li key={i} className="flex items-center text-sm text-muted-foreground">
                            <Check className="h-3 w-3 mr-2 text-green-500" />
                            <span>{feature}</span>
                          </li>
                        ))}
                      </ul>
                    </div>
                    <Button 
                      variant="ghost"
                      onClick={() => setSelectedPlan(null)}
                    >
                      Remove
                    </Button>
                  </div>
                </CardContent>
              </Card>
            ) : (
              <Card>
                <CardContent className="pt-6">
                  <h2 className="text-lg font-medium mb-4">No Plan Selected</h2>
                  <p className="text-muted-foreground">
                    You haven't selected a plan yet. <Link to="/pricing" className="text-primary underline">View available plans</Link>
                  </p>
                </CardContent>
              </Card>
            )}
            
            <Card>
              <CardContent className="pt-6">
                <h2 className="text-lg font-medium mb-4">Selected Add-ons</h2>
                
                {selectedAddons.length > 0 ? (
                  <div className="space-y-4">
                    {selectedAddons.map((addon) => (
                      <div key={addon.id} className="flex items-center justify-between py-2 border-b last:border-0">
                        <div>
                          <h3 className="font-medium">{addon.name}</h3>
                          <p className="text-sm text-muted-foreground">
                            ${(addon.price/100).toFixed(2)}/month
                          </p>
                        </div>
                        
                        <div className="flex items-center gap-4">
                          <div className="flex items-center gap-2">
                            <Button
                              variant="outline"
                              size="icon"
                              className="h-8 w-8"
                              onClick={() => handleQuantityChange(addon.id, -1)}
                            >
                              -
                            </Button>
                            <span className="min-w-[30px] text-center">
                              {addon.quantity}
                            </span>
                            <Button
                              variant="outline"
                              size="icon"
                              className="h-8 w-8"
                              onClick={() => handleQuantityChange(addon.id, 1)}
                            >
                              +
                            </Button>
                          </div>
                          
                          <span className="font-medium w-20 text-right">
                            ${((addon.price * addon.quantity)/100).toFixed(2)}
                          </span>
                          
                          <Button
                            variant="ghost"
                            size="sm"
                            onClick={() => handleRemoveAddon(addon.id)}
                          >
                            Remove
                          </Button>
                        </div>
                      </div>
                    ))}
                  </div>
                ) : (
                  <p className="text-muted-foreground">No add-ons selected</p>
                )}
                
                {/* Available Add-ons */}
                <div className="mt-6">
                  <h3 className="text-md font-medium mb-3">Available Add-ons</h3>
                  <div className="grid grid-cols-1 sm:grid-cols-3 gap-4">
                    {addons.map((addon) => {
                      const isSelected = selectedAddons.some(a => a.id === addon.id);
                      
                      return (
                        <div 
                          key={addon.id} 
                          className={`border rounded-lg p-4 flex flex-col transition-all ${
                            isSelected ? 'border-primary bg-primary/10' : 'hover:border-primary'
                          }`}
                        >
                          <h4 className="font-medium">{addon.name}</h4>
                          <p className="text-sm text-muted-foreground mb-2">{addon.description}</p>
                          <div className="mt-auto flex justify-between items-center">
                            <span className="font-medium">${(addon.price/100).toFixed(2)}/month</span>
                            <Button 
                              variant={isSelected ? "outline" : "default"}
                              size="sm"
                              className={isSelected ? "gap-2" : ""}
                              onClick={() => {
                                if (!isSelected) {
                                  handleAddAddon(addon);
                                } else {
                                  const existing = selectedAddons.find(a => a.id === addon.id);
                                  if (existing) {
                                    handleQuantityChange(addon.id, 1);
                                  }
                                }
                              }}
                            >
                              {isSelected ? (
                                <>
                                  <Check className="h-4 w-4" />
                                  Add More
                                </>
                              ) : (
                                "Add"
                              )}
                            </Button>
                          </div>
                        </div>
                      );
                    })}
                  </div>
                </div>
              </CardContent>
            </Card>
          </div>
          
          {/* Order Summary */}
          <div className="md:col-span-1">
            <Card className="sticky top-24">
              <CardContent className="pt-6">
                <h2 className="text-lg font-medium mb-4">Order Summary</h2>
                <div className="space-y-2">
                  {selectedPlan && (
                    <div className="flex justify-between text-sm">
                      <span>{selectedPlan.name} Plan</span>
                      <span>${(selectedPlan.price/100).toFixed(2)}</span>
                    </div>
                  )}
                  
                  {selectedAddons.map((addon) => (
                    <div key={addon.id} className="flex justify-between text-sm">
                      <span>
                        {addon.name} {addon.quantity > 1 && `× ${addon.quantity}`}
                      </span>
                      <span>${((addon.price * addon.quantity)/100).toFixed(2)}</span>
                    </div>
                  ))}
                  
                  <Separator className="my-3" />
                  
                  <div className="flex justify-between font-medium">
                    <span>Total</span>
                    <span>${(total/100).toFixed(2)}</span>
                  </div>
                  
                  {selectedPlan && (
                    <p className="text-sm text-muted-foreground mt-2">
                      You'll be billed ${(total/100).toFixed(2)} now, and subscription amounts will be charged recurring until canceled.
                    </p>
                  )}
                  
                  <div className="pt-4 mt-2">
                    <Button 
                      className="w-full" 
                      onClick={handleCheckout}
                      disabled={isLoading || (total === 0)}
                    >
                      {isLoading ? "Processing..." : "Proceed to Payment"}
                    </Button>
                    <Button 
                      variant="outline"
                      className="w-full mt-2" 
                      onClick={() => navigate("/pricing")}
                    >
                      Cancel
                    </Button>
                  </div>
                </div>
              </CardContent>
            </Card>
          </div>
        </div>
      </div>
    </PlaceholderLayout>
  );
};

export default CheckoutPage;
