
import * as React from "react";
import { ShoppingCart } from "lucide-react";
import { Button } from "@/components/ui/button";
import {
  Drawer,
  DrawerClose,
  DrawerContent,
  DrawerDescription,
  DrawerFooter,
  DrawerHeader,
  DrawerTitle,
  DrawerTrigger,
} from "@/components/ui/drawer";
import { Badge } from "@/components/ui/badge";
import { useCart } from "@/context/CartContext";
import { toast } from "@/components/ui/sonner";
import { useAuth } from "@/context/AuthContext";
import { supabase } from "@/integrations/supabase/client";

export function CartDrawer() {
  const { cart, removeFromCart, clearCart } = useCart();
  const { user } = useAuth();
  const [isLoading, setIsLoading] = React.useState(false);

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
  
  const totalItems = cart.reduce((acc, item) => acc + (item.quantity || 1), 0);
  const subtotal = cart.reduce((acc, item) => acc + ((item.price || 0) * (item.quantity || 1)), 0);

  return (
    <Drawer>
      <DrawerTrigger asChild>
        <Button variant="outline" size="icon" className="relative">
          <ShoppingCart className="h-5 w-5" />
          {totalItems > 0 && (
            <Badge className="absolute -top-2 -right-2 px-1 min-w-5 h-5 flex items-center justify-center">
              {totalItems}
            </Badge>
          )}
        </Button>
      </DrawerTrigger>
      <DrawerContent>
        <div className="mx-auto w-full max-w-sm">
          <DrawerHeader>
            <DrawerTitle>Your Cart</DrawerTitle>
            <DrawerDescription>
              {cart.length === 0 ? "Your cart is empty" : `${cart.length} item(s) in your cart`}
            </DrawerDescription>
          </DrawerHeader>
          
          {cart.length > 0 ? (
            <>
              <div className="p-4 max-h-80 overflow-auto">
                {cart.map((item) => (
                  <div key={item.id} className="flex items-center justify-between py-4 border-b">
                    <div>
                      <h4 className="font-medium">{item.name}</h4>
                      <div className="text-sm text-muted-foreground">
                        ${(item.price/100).toFixed(2)} × {item.quantity || 1}
                      </div>
                    </div>
                    <div className="flex items-center gap-2">
                      <div className="font-medium">
                        ${((item.price * (item.quantity || 1))/100).toFixed(2)}
                      </div>
                      <Button
                        variant="ghost"
                        size="sm"
                        onClick={() => removeFromCart(item.id)}
                      >
                        Remove
                      </Button>
                    </div>
                  </div>
                ))}
              </div>
              
              <div className="px-4 py-2 border-t">
                <div className="flex justify-between py-2">
                  <span className="font-medium">Subtotal</span>
                  <span className="font-medium">${(subtotal/100).toFixed(2)}</span>
                </div>
              </div>
              
              <DrawerFooter>
                <Button onClick={handleCheckout} disabled={isLoading} className="w-full">
                  {isLoading ? "Processing..." : "Checkout"}
                </Button>
                <div className="flex justify-between gap-2">
                  <Button variant="outline" onClick={clearCart} className="w-1/2">
                    Clear Cart
                  </Button>
                  <DrawerClose asChild>
                    <Button variant="outline" className="w-1/2">Continue Shopping</Button>
                  </DrawerClose>
                </div>
              </DrawerFooter>
            </>
          ) : (
            <div className="flex flex-col items-center justify-center px-4 py-8">
              <ShoppingCart className="h-12 w-12 text-muted-foreground mb-4" />
              <p className="text-muted-foreground mb-4">Your cart is empty</p>
              <DrawerClose asChild>
                <Button variant="outline">Browse Plans</Button>
              </DrawerClose>
            </div>
          )}
        </div>
      </DrawerContent>
    </Drawer>
  );
}
