
import { useState } from "react";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { useRobots } from "@/hooks/useRobots";
import { Copy, Check } from "lucide-react";
import { useToast } from "@/hooks/use-toast";

export function ApiKeySettings() {
  const { apiKey } = useRobots();
  const { toast } = useToast();
  const [copied, setCopied] = useState(false);

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
        <div className="flex items-center gap-2">
          <div className="bg-muted p-2 rounded-md text-xs font-mono flex-1 truncate">
            {apiKey || "Loading..."}
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
        <p className="text-xs text-muted-foreground mt-2">
          You'll need both this API key and a Robot ID to send data
        </p>
      </CardContent>
    </Card>
  );
}
