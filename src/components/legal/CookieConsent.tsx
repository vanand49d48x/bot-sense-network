
import { useState, useEffect } from "react";
import { Button } from "@/components/ui/button";
import { 
  Sheet,
  SheetContent,
  SheetHeader,
  SheetTitle,
  SheetDescription,
  SheetFooter
} from "@/components/ui/sheet";
import { X } from "lucide-react";

// Key to store cookie consent in localStorage
const COOKIE_CONSENT_KEY = "robometrics-cookie-consent";

export type CookieConsent = {
  essential: boolean; // Always true, can't be disabled
  analytics: boolean;
  marketing: boolean;
  preferences: boolean;
};

// Default state when a user hasn't made a choice yet
const DEFAULT_CONSENT: CookieConsent = {
  essential: true,
  analytics: false,
  marketing: false,
  preferences: false,
};

export const CookieConsent = () => {
  const [open, setOpen] = useState(false);
  const [consent, setConsent] = useState<CookieConsent>(DEFAULT_CONSENT);

  // Load consent from localStorage on mount
  useEffect(() => {
    const savedConsent = localStorage.getItem(COOKIE_CONSENT_KEY);
    
    if (!savedConsent) {
      // If no consent is stored, show the banner
      setOpen(true);
    } else {
      try {
        // Parse the stored consent
        setConsent(JSON.parse(savedConsent));
      } catch (e) {
        console.error("Error parsing cookie consent:", e);
        setOpen(true);
      }
    }
  }, []);

  // Handlers for consent actions
  const handleAcceptAll = () => {
    const fullConsent: CookieConsent = {
      essential: true,
      analytics: true, 
      marketing: true,
      preferences: true,
    };
    setConsent(fullConsent);
    saveConsent(fullConsent);
    setOpen(false);
  };

  const handleRejectAll = () => {
    // Essential cookies are always accepted
    const minimalConsent: CookieConsent = {
      essential: true,
      analytics: false,
      marketing: false,
      preferences: false,
    };
    setConsent(minimalConsent);
    saveConsent(minimalConsent);
    setOpen(false);
  };

  const handleCustomize = (updatedConsent: Partial<CookieConsent>) => {
    const newConsent = { ...consent, ...updatedConsent };
    setConsent(newConsent);
    saveConsent(newConsent);
  };

  const saveConsent = (consentData: CookieConsent) => {
    // Store the consent in localStorage
    localStorage.setItem(COOKIE_CONSENT_KEY, JSON.stringify(consentData));
    
    // Here you would typically trigger any analytics or tracking based on consent
    // For example, if consent.analytics is true, initialize Google Analytics
    
    console.log("Cookie consent saved:", consentData);
  };

  // Allow users to reopen the cookie banner
  const reopenConsentBanner = () => {
    setOpen(true);
  };

  return (
    <>
      <Sheet open={open} onOpenChange={setOpen}>
        <SheetContent side="bottom" className="h-auto max-w-full p-0">
          <div className="flex flex-col gap-4 p-6">
            <SheetHeader className="mb-2">
              <div className="flex justify-between items-center">
                <SheetTitle>Cookie Preferences</SheetTitle>
              </div>
              <SheetDescription>
                We use cookies to enhance your browsing experience, serve personalized content, and analyze our traffic.
                Read our <a href="/cookies" className="text-primary underline">Cookie Policy</a> to learn more.
              </SheetDescription>
            </SheetHeader>
            
            <div className="space-y-4">
              <div className="flex items-center gap-2">
                <input 
                  type="checkbox" 
                  id="essential-cookies" 
                  checked={consent.essential} 
                  disabled 
                  className="rounded border-gray-300"
                />
                <div>
                  <label htmlFor="essential-cookies" className="font-medium">Essential</label>
                  <p className="text-muted-foreground text-sm">Required for basic site functionality. Cannot be disabled.</p>
                </div>
              </div>
              
              <div className="flex items-center gap-2">
                <input 
                  type="checkbox" 
                  id="analytics-cookies" 
                  checked={consent.analytics} 
                  onChange={(e) => handleCustomize({ analytics: e.target.checked })} 
                  className="rounded border-gray-300"
                />
                <div>
                  <label htmlFor="analytics-cookies" className="font-medium">Analytics</label>
                  <p className="text-muted-foreground text-sm">Help us improve through anonymous site usage data.</p>
                </div>
              </div>
              
              <div className="flex items-center gap-2">
                <input 
                  type="checkbox" 
                  id="marketing-cookies" 
                  checked={consent.marketing} 
                  onChange={(e) => handleCustomize({ marketing: e.target.checked })} 
                  className="rounded border-gray-300"
                />
                <div>
                  <label htmlFor="marketing-cookies" className="font-medium">Marketing</label>
                  <p className="text-muted-foreground text-sm">Allow us to provide personalized advertisements.</p>
                </div>
              </div>
              
              <div className="flex items-center gap-2">
                <input 
                  type="checkbox" 
                  id="preferences-cookies" 
                  checked={consent.preferences} 
                  onChange={(e) => handleCustomize({ preferences: e.target.checked })} 
                  className="rounded border-gray-300"
                />
                <div>
                  <label htmlFor="preferences-cookies" className="font-medium">Preferences</label>
                  <p className="text-muted-foreground text-sm">Remember your settings and personalization choices.</p>
                </div>
              </div>
            </div>
            
            <SheetFooter className="flex sm:justify-between gap-2 border-t pt-4">
              <div className="flex flex-col sm:flex-row gap-2">
                <Button variant="outline" onClick={handleRejectAll}>
                  Reject All
                </Button>
                <Button onClick={handleAcceptAll}>
                  Accept All
                </Button>
              </div>
              <Button 
                variant="secondary" 
                onClick={() => setOpen(false)}
                className="sm:ml-auto"
              >
                Save Preferences
              </Button>
            </SheetFooter>
          </div>
        </SheetContent>
      </Sheet>
      
      {/* Small fixed button to reopen consent banner if needed */}
      {!open && (
        <button
          onClick={reopenConsentBanner}
          className="fixed bottom-4 left-4 bg-muted p-2 rounded-full shadow-md hover:bg-muted/90 z-50"
          aria-label="Cookie Settings"
        >
          <span className="sr-only">Cookie Settings</span>
          <svg
            xmlns="http://www.w3.org/2000/svg"
            width="18"
            height="18"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <path d="M12 2a10 10 0 1 0 10 10 4 4 0 0 1-5-5 4 4 0 0 1-5-5" />
            <circle cx="7" cy="7" r="1" />
            <circle cx="15" cy="15" r="1" />
            <circle cx="12" cy="18" r="1" />
          </svg>
        </button>
      )}
    </>
  );
};

export default CookieConsent;
