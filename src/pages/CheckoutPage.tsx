
import React, { useEffect, useState } from 'react';
import PlaceholderLayout from "@/components/layout/PlaceholderLayout";
import { useCart } from "@/context/CartContext";
import { Button } from "@/components/ui/button";
import { Card, CardContent } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import { Trash2, Plus, Minus, Check } from "lucide-react";
import { useAuth } from "@/context/AuthContext";
import { supabase } from "@/integrations/supabase/client";
import { toast } from "@/components/ui/sonner";
import { useNavigate } from "react-router-dom";
import { addons } from "@/data/pricingData";

const CheckoutPage = () => {
  const { cart, removeFromCart, updateQuantity, clearCart, addToCart, isInCart } = useCart();
  const { user } = useAuth();
  const navigate = useNavigate();
  const [isLoading, setIsLoading] = useState(false);

  // Redirect to login if not logged in
  useEffect(() => {
    if (!user) {
      toast.error("Please log in to proceed with checkout");
      navigate("/auth", { state: { redirectUrl: "/checkout" } });
    }
  }, [user, navigate]);

  const handleCheckout = async () => {
    if (!user) {
      toast.error("Please log in to proceed with checkout");
      return;
    }
    
    setIsLoading(true);
    
    try {
      const { data, error } = await supabase.functions.invoke('create-checkout', {
        body: {
          items: cart,
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

  // Calculate subtotal
  const subtotal = cart.reduce((acc, item) => 
    acc + ((item.price || 0) * (item.quantity || 1)), 0
  );

  // Check if we have a subscription in the cart
  const hasSubscription = cart.some(item => item.type === 'subscription');

  return (
    <PlaceholderLayout title="Checkout">
      <div className="max-w-5xl mx-auto px-4">
        <h1 className="text-2xl font-bold mb-8">Checkout</h1>
        
        {cart.length > 0 ? (
          <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
            {/* Cart Items */}
            <div className="md:col-span-2 space-y-6">
              <Card>
                <CardContent className="pt-6">
                  <h2 className="text-lg font-medium mb-4">Your Cart</h2>
                  
                  {cart.map((item) => (
                    <div key={item.id} className="flex items-center justify-between py-4 border-b last:border-0">
                      <div className="flex-1">
                        <h3 className="font-medium">{item.name}</h3>
                        <p className="text-sm text-muted-foreground">
                          ${(item.price/100).toFixed(2)}
                          {item.type === 'subscription' && item.period && `/${item.period}`}
                        </p>
                      </div>
                      
                      {item.type === 'addon' && (
                        <div className="flex items-center gap-2 mr-4">
                          <Button
                            variant="outline"
                            size="icon"
                            className="h-8 w-8"
                            onClick={() => updateQuantity(item.id, (item.quantity || 1) - 1)}
                          >
                            <Minus className="h-4 w-4" />
                          </Button>
                          <span className="min-w-[30px] text-center">
                            {item.quantity || 1}
                          </span>
                          <Button
                            variant="outline"
                            size="icon"
                            className="h-8 w-8"
                            onClick={() => updateQuantity(item.id, (item.quantity || 1) + 1)}
                          >
                            <Plus className="h-4 w-4" />
                          </Button>
                        </div>
                      )}
                      
                      <div className="flex items-center gap-4">
                        <span className="font-medium">
                          ${((item.price * (item.quantity || 1))/100).toFixed(2)}
                        </span>
                        <Button
                          variant="ghost"
                          size="icon"
                          onClick={() => removeFromCart(item.id)}
                        >
                          <Trash2 className="h-4 w-4" />
                        </Button>
                      </div>
                    </div>
                  ))}
                </CardContent>
              </Card>
              
              {/* Add-ons section */}
              <Card>
                <CardContent className="pt-6">
                  <h2 className="text-lg font-medium mb-4">Popular Add-ons</h2>
                  <div className="grid grid-cols-1 sm:grid-cols-3 gap-4">
                    {addons.map((addon) => {
                      const inCart = isInCart(addon.id);
                      
                      return (
                        <div 
                          key={addon.id} 
                          className={`border rounded-lg p-4 flex flex-col transition-all ${
                            inCart ? 'border-primary bg-primary/10' : 'hover:border-primary'
                          }`}
                        >
                          <h3 className="font-medium">{addon.name}</h3>
                          <p className="text-sm text-muted-foreground mb-2">{addon.description}</p>
                          <div className="mt-auto flex justify-between items-center">
                            <span className="font-medium">${(addon.price/100).toFixed(2)}/month</span>
                            <Button 
                              variant={inCart ? "outline" : "default"}
                              size="sm"
                              className={inCart ? "gap-2" : ""}
                              onClick={() => {
                                if (!inCart) {
                                  addToCart({
                                    id: addon.id,
                                    name: addon.name,
                                    price: addon.price,
                                    type: 'addon',
                                    quantity: 1
                                  });
                                } else {
                                  removeFromCart(addon.id);
                                }
                              }}
                            >
                              {inCart ? (
                                <>
                                  <Check className="h-4 w-4" />
                                  Added
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
                </CardContent>
              </Card>
            </div>
            
            {/* Order Summary */}
            <div>
              <Card className="sticky top-24">
                <CardContent className="pt-6">
                  <h2 className="text-lg font-medium mb-4">Order Summary</h2>
                  <div className="space-y-2">
                    {cart.map((item) => (
                      <div key={item.id} className="flex justify-between text-sm">
                        <span>
                          {item.name} {item.quantity && item.quantity > 1 ? `× ${item.quantity}` : ""}
                        </span>
                        <span>${((item.price * (item.quantity || 1))/100).toFixed(2)}</span>
                      </div>
                    ))}
                    
                    <Separator className="my-3" />
                    
                    <div className="flex justify-between font-medium">
                      <span>Subtotal</span>
                      <span>${(subtotal/100).toFixed(2)}</span>
                    </div>
                    
                    {hasSubscription && (
                      <p className="text-sm text-muted-foreground mt-2">
                        You'll be billed ${(subtotal/100).toFixed(2)} now, and subscription amounts will be charged recurring until canceled.
                      </p>
                    )}
                    
                    <div className="pt-4 mt-2">
                      <Button 
                        className="w-full" 
                        onClick={handleCheckout}
                        disabled={isLoading || cart.length === 0}
                      >
                        {isLoading ? "Processing..." : "Proceed to Payment"}
                      </Button>
                      <Button 
                        variant="outline"
                        className="w-full mt-2" 
                        onClick={() => {
                          clearCart();
                          navigate("/pricing");
                        }}
                      >
                        Cancel Order
                      </Button>
                    </div>
                  </div>
                </CardContent>
              </Card>
            </div>
          </div>
        ) : (
          <div className="text-center py-12">
            <h2 className="text-xl font-semibold mb-4">Your cart is empty</h2>
            <p className="text-muted-foreground mb-8">Add items from our pricing page to get started</p>
            <Button onClick={() => navigate("/pricing")}>
              Browse Plans
            </Button>
          </div>
        )}
      </div>
    </PlaceholderLayout>
  );
};

export default CheckoutPage;
