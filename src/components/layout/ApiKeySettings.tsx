
import { useState } from "react";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { useRobots } from "@/hooks/useRobots";
import { Copy, Check, RefreshCw } from "lucide-react";
import { useToast } from "@/hooks/use-toast";
import { supabase } from "@/integrations/supabase/client";
import { useAuth } from "@/context/AuthContext";

export function ApiKeySettings() {
  const { apiKey, fetchRobots } = useRobots();
  const { toast } = useToast();
  const { session } = useAuth();
  const [copied, setCopied] = useState(false);
  const [regenerating, setRegenerating] = useState(false);

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
        title: "Failed to copy",
        description: "Could not copy API key to clipboard. Please try again.",
        variant: "destructive"
      });
    }
  };
  
  const regenerateApiKey = async () => {
    if (!session) return;
    
    try {
      setRegenerating(true);
      const newApiKey = crypto.randomUUID().replace(/-/g, '') + crypto.randomUUID().replace(/-/g, '');
      
      const { error } = await supabase
        .from('profiles')
        .update({ api_key: newApiKey })
        .eq('id', session.user.id);
        
      if (error) throw error;
      
      // Refetch data to update the API key
      await fetchRobots();
      
      toast({
        title: "API key regenerated",
        description: "A new API key has been generated for your account."
      });
    } catch (error: any) {
      console.error("Failed to regenerate API key:", error);
      toast({
        title: "Failed to regenerate API key",
        description: error.message || "An unexpected error occurred",
        variant: "destructive"
      });
    } finally {
      setRegenerating(false);
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
            {apiKey || "Generating API key..."}
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
        <div className="flex flex-col space-y-2">
          <p className="text-xs text-muted-foreground">
            You'll need both this API key and a Robot ID to send data
          </p>
          <Button
            variant="outline"
            size="sm"
            className="w-full mt-2"
            onClick={regenerateApiKey}
            disabled={regenerating || !session}
          >
            {regenerating ? (
              <>
                <RefreshCw className="h-4 w-4 mr-2 animate-spin" />
                Regenerating...
              </>
            ) : (
              <>
                <RefreshCw className="h-4 w-4 mr-2" />
                Regenerate Key
              </>
            )}
          </Button>
        </div>
      </CardContent>
    </Card>
  );
}
