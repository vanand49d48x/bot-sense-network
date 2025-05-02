
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { 
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";
import { Check, AlertTriangle, Battery, Thermometer, BellRing } from "lucide-react";

export function EmailNotifications() {
  return (
    <Card>
      <CardHeader>
        <CardTitle>Email Notification System</CardTitle>
        <CardDescription>
          RoboMetrics can send automated email notifications based on robot status and telemetry data. 
          Use the Alerts page to configure notification preferences.
        </CardDescription>
      </CardHeader>
      <CardContent className="space-y-6">
        <div className="space-y-2">
          <h3 className="text-lg font-medium">Notification Types</h3>
          <p className="text-sm text-muted-foreground">
            The platform can send notifications for the following events:
          </p>
          
          <Table>
            <TableHeader>
              <TableRow>
                <TableHead>Event Type</TableHead>
                <TableHead>Description</TableHead>
                <TableHead>Default Threshold</TableHead>
              </TableRow>
            </TableHeader>
            <TableBody>
              <TableRow>
                <TableCell className="font-medium">
                  <div className="flex items-center gap-2">
                    <Battery className="w-4 h-4 text-yellow-500" />
                    <span>Low Battery</span>
                  </div>
                </TableCell>
                <TableCell>Alerts when robot battery level drops below threshold</TableCell>
                <TableCell>20%</TableCell>
              </TableRow>
              <TableRow>
                <TableCell className="font-medium">
                  <div className="flex items-center gap-2">
                    <BellRing className="w-4 h-4 text-red-500" />
                    <span>Robot Offline</span>
                  </div>
                </TableCell>
                <TableCell>Alerts when robot becomes unreachable</TableCell>
                <TableCell>5 minutes with no ping</TableCell>
              </TableRow>
              <TableRow>
                <TableCell className="font-medium">
                  <div className="flex items-center gap-2">
                    <Thermometer className="w-4 h-4 text-orange-500" />
                    <span>High Temperature</span>
                  </div>
                </TableCell>
                <TableCell>Alerts when robot temperature exceeds threshold</TableCell>
                <TableCell>35°C</TableCell>
              </TableRow>
              <TableRow>
                <TableCell className="font-medium">
                  <div className="flex items-center gap-2">
                    <AlertTriangle className="w-4 h-4 text-red-500" />
                    <span>Error State</span>
                  </div>
                </TableCell>
                <TableCell>Alerts when robot reports error status</TableCell>
                <TableCell>Immediate</TableCell>
              </TableRow>
            </TableBody>
          </Table>
        </div>
        
        <div className="p-4 border rounded-md bg-muted/40">
          <h3 className="text-lg font-medium mb-2">Setting Up Email Notifications</h3>
          <ol className="list-decimal ml-5 space-y-2 text-sm">
            <li>Go to the <strong>Alerts</strong> page in the sidebar</li>
            <li>Click on any alert card to filter alerts by type</li>
            <li>Use the "Email" button to send notification for a specific robot</li>
            <li>For notification preferences, visit your <strong>Profile</strong> page</li>
          </ol>
        </div>

        <Tabs defaultValue="api">
          <TabsList className="mb-4">
            <TabsTrigger value="api">API Integration</TabsTrigger>
            <TabsTrigger value="custom">Custom Email Templates</TabsTrigger>
          </TabsList>
          <TabsContent value="api" className="space-y-4">
            <p className="text-sm text-muted-foreground">
              You can trigger email notifications programmatically using our API:
            </p>
            <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
              {`
// Example API call to send a custom notification
fetch("https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/send-notification", {
  method: "POST",
  headers: {
    "Content-Type": "application/json",
    "api-key": "YOUR_API_KEY"
  },
  body: JSON.stringify({
    robotId: "YOUR_ROBOT_ID",
    type: "CUSTOM", // or "LOW_BATTERY", "OFFLINE", "HIGH_TEMP", "ERROR"
    message: "Robot arm requires maintenance",
    severity: "WARNING" // "INFO", "WARNING", or "CRITICAL"
  })
})
.then(response => response.json())
.then(data => console.log(data))
.catch(error => console.error("Error:", error));
              `}
            </pre>
            <p className="text-sm text-muted-foreground mt-4">
              <strong>Note:</strong> The email notification edge function will be available soon. Check back for updates.
            </p>
          </TabsContent>
          <TabsContent value="custom">
            <div className="space-y-4">
              <p className="text-sm text-muted-foreground">
                Enterprise users can customize email notification templates. Contact us for more information about enterprise features.
              </p>
              <div className="bg-muted p-4 rounded-md">
                <h4 className="font-medium mb-2 text-sm">Available Variables in Templates</h4>
                <ul className="list-disc ml-5 space-y-1 text-xs">
                  <li><code>{'{{robotName}}'}</code> - Name of the robot</li>
                  <li><code>{'{{robotId}}'}</code> - ID of the robot</li>
                  <li><code>{'{{alertType}}'}</code> - Type of the alert</li>
                  <li><code>{'{{alertMessage}}'}</code> - Alert message</li>
                  <li><code>{'{{timestamp}}'}</code> - Time when the alert was generated</li>
                  <li><code>{'{{batteryLevel}}'}</code> - Current battery level</li>
                  <li><code>{'{{temperature}}'}</code> - Current temperature</li>
                  <li><code>{'{{location}}'}</code> - Current location (if available)</li>
                </ul>
              </div>
            </div>
          </TabsContent>
        </Tabs>
      </CardContent>
    </Card>
  );
}
