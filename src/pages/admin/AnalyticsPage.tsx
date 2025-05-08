
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";

export default function AnalyticsPage() {
  return (
    <div className="space-y-6">
      <div>
        <h1 className="text-3xl font-bold tracking-tight">Analytics</h1>
        <p className="text-muted-foreground">
          System-wide analytics and reporting
        </p>
      </div>

      <Card>
        <CardHeader>
          <CardTitle>Analytics Dashboard</CardTitle>
          <CardDescription>
            Coming soon - Advanced system analytics and reporting
          </CardDescription>
        </CardHeader>
        <CardContent className="min-h-[300px] flex items-center justify-center text-muted-foreground">
          Analytics functionality is under development
        </CardContent>
      </Card>
    </div>
  );
}
