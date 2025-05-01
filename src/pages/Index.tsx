
import { MainLayout } from "@/components/layout/MainLayout";
import { Dashboard } from "@/components/dashboard/Dashboard";
import { useAuth } from "@/context/AuthContext";

const Index = () => {
  const { user, loading } = useAuth();

  // Add a loading state to make it clear when content is being loaded
  if (loading) {
    return (
      <div className="flex h-screen items-center justify-center bg-background p-4">
        <div className="text-center">
          <h2 className="text-2xl font-semibold mb-2">Loading dashboard...</h2>
          <p className="text-muted-foreground">Please wait while we fetch your robot data</p>
        </div>
      </div>
    );
  }

  // Show a message if user isn't authenticated
  if (!user) {
    return (
      <div className="flex h-screen items-center justify-center bg-background p-4">
        <div className="text-center">
          <h2 className="text-2xl font-semibold mb-2">Authentication required</h2>
          <p className="text-muted-foreground">Please sign in to access the dashboard</p>
        </div>
      </div>
    );
  }

  return (
    <MainLayout>
      <Dashboard />
    </MainLayout>
  );
};

export default Index;
