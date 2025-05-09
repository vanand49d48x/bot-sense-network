
import { useState } from "react";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { useRobots } from "@/hooks/useRobots";
import { Copy, Check, RefreshCw } from "lucide-react";
import { useToast } from "@/hooks/use-toast";
import { supabase } from "@/integrations/supabase/client";
import { useAuth } from "@/context/AuthContext";

export function ApiKeySettings() {
  const { apiKey } = useRobots();
  const { toast } = useToast();
  const { session } = useAuth();
  const [copied, setCopied] = useState(false);
  const [isRegenerating, setIsRegenerating] = useState(false);

  const copyToClipboard = async () => {
    if (!apiKey) return;
    
    try {
      await navigator.clipboard.writeText(apiKey);
      setCopied(true);
      toast({
        title: "API key copied",
        description: "The API key has been copied to your clipboard."
      });
      
      setTimeout(() => setCopied(false), 2000);
    } catch (error) {
      console.error("Failed to copy:", error);
      toast({
        title: "Copy failed",
        description: "Could not copy API key to clipboard",
        variant: "destructive"
      });
    }
  };
  
  const regenerateApiKey = async () => {
    if (!session || isRegenerating) return;
    
    try {
      setIsRegenerating(true);
      const newApiKey = crypto.randomUUID().replace(/-/g, '') + crypto.randomUUID().replace(/-/g, '');
      
      const { error } = await supabase
        .from('profiles')
        .update({ api_key: newApiKey })
        .eq('id', session.user.id);
      
      if (error) {
        throw error;
      }
      
      toast({
        title: "API key regenerated",
        description: "Your API key has been successfully regenerated."
      });
      
      // Force page refresh to update the API key display
      window.location.reload();
    } catch (error: any) {
      console.error("Failed to regenerate API key:", error);
      toast({
        title: "Regeneration failed",
        description: error.message || "Could not regenerate API key",
        variant: "destructive"
      });
    } finally {
      setIsRegenerating(false);
    }
  };

  return (
    <Card>
      <CardHeader>
        <CardTitle>API Key</CardTitle>
        <CardDescription>
          Use this API key for all your robots when sending telemetry data
        </CardDescription>
      </CardHeader>
      <CardContent>
        <div className="flex items-center gap-2 mb-2">
          <div className="bg-muted p-2 rounded-md text-xs font-mono flex-1 truncate">
            {apiKey || "No API key available"}
          </div>
          <Button 
            variant="outline" 
            size="icon" 
            onClick={copyToClipboard}
            disabled={!apiKey}
          >
            {copied ? <Check size={16} /> : <Copy size={16} />}
          </Button>
        </div>
        <div className="flex justify-between items-center">
          <p className="text-xs text-muted-foreground mt-2">
            You'll need both this API key and a Robot ID to send data
          </p>
          <Button
            variant="outline"
            size="sm"
            onClick={regenerateApiKey}
            disabled={isRegenerating}
            className="mt-2"
          >
            <RefreshCw size={14} className={`mr-1 ${isRegenerating ? 'animate-spin' : ''}`} />
            Regenerate
          </Button>
        </div>
      </CardContent>
    </Card>
  );
}
