import React, { useEffect, useState } from 'react';
import { supabase } from '@/integrations/supabase/client';
import { useToast } from '@/hooks/use-toast';
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";

type Subscription = {
  id: string;
  user_id: string;
  plan_name: string | null;
  status: string;
  current_period_end: string | null;
  trial_started_at: string | null;
  trial_ended_at: string | null;
  trial_status: string | null;
  created_at: string;
  user: {
    email: string;
    profile: {
      first_name: string | null;
      last_name: string | null;
    } | null;
  } | null;
};

const SubscriptionManagement = () => {
  const [subscriptions, setSubscriptions] = useState<Subscription[]>([]);
  const [loading, setLoading] = useState(true);
  const { toast } = useToast();

  useEffect(() => {
    fetchSubscriptions();
  }, []);

  const fetchSubscriptions = async () => {
    try {
      setLoading(true);
      const { data: subscriptions, error } = await supabase
        .from('subscriptions')
        .select(`
          *,
          user:user_id (
            id,
            first_name,
            last_name
          )
        `);

      if (error) throw error;
      setSubscriptions(subscriptions || []);
    } catch (error: any) {
      toast({
        title: "Error fetching subscriptions",
        description: error.message,
        variant: "destructive",
      });
    } finally {
      setLoading(false);
    }
  };

  const getStatusColor = (status: string) => {
    switch (status.toLowerCase()) {
      case 'active':
        return 'bg-green-500';
      case 'canceled':
        return 'bg-red-500';
      case 'trialing':
        return 'bg-blue-500';
      case 'past_due':
        return 'bg-yellow-500';
      default:
        return 'bg-gray-500';
    }
  };

  const handleCancelSubscription = async (subscriptionId: string) => {
    try {
      const { error } = await supabase
        .from('subscriptions')
        .update({ status: 'canceled' })
        .eq('id', subscriptionId);

      if (error) throw error;

      toast({
        title: "Subscription canceled",
        description: "The subscription has been successfully canceled.",
      });
      fetchSubscriptions();
    } catch (error: any) {
      toast({
        title: "Error canceling subscription",
        description: error.message,
        variant: "destructive",
      });
    }
  };

  if (loading) {
    return <div>Loading subscriptions...</div>;
  }

  return (
    <div className="space-y-4">
      <div className="flex justify-between items-center">
        <h2 className="text-2xl font-bold">Subscriptions</h2>
      </div>

      <Table>
        <TableHeader>
          <TableRow>
            <TableHead>User</TableHead>
            <TableHead>Plan</TableHead>
            <TableHead>Status</TableHead>
            <TableHead>Current Period End</TableHead>
            <TableHead>Trial Status</TableHead>
            <TableHead>Created</TableHead>
            <TableHead>Actions</TableHead>
          </TableRow>
        </TableHeader>
        <TableBody>
          {subscriptions.map((subscription) => (
            <TableRow key={subscription.id}>
              <TableCell>
                {subscription.user?.profile?.first_name} {subscription.user?.profile?.last_name}
                <br />
                <span className="text-sm text-gray-500">{subscription.user?.email}</span>
              </TableCell>
              <TableCell>{subscription.plan_name || 'N/A'}</TableCell>
              <TableCell>
                <Badge className={getStatusColor(subscription.status)}>
                  {subscription.status}
                </Badge>
              </TableCell>
              <TableCell>
                {subscription.current_period_end
                  ? new Date(subscription.current_period_end).toLocaleDateString()
                  : 'N/A'}
              </TableCell>
              <TableCell>
                {subscription.trial_status ? (
                  <div>
                    <Badge className="bg-blue-500">
                      {subscription.trial_status}
                    </Badge>
                    <br />
                    <span className="text-sm text-gray-500">
                      {subscription.trial_started_at && (
                        <>Started: {new Date(subscription.trial_started_at).toLocaleDateString()}</>
                      )}
                      {subscription.trial_ended_at && (
                        <>Ended: {new Date(subscription.trial_ended_at).toLocaleDateString()}</>
                      )}
                    </span>
                  </div>
                ) : (
                  'No trial'
                )}
              </TableCell>
              <TableCell>
                {new Date(subscription.created_at).toLocaleDateString()}
              </TableCell>
              <TableCell>
                <div className="space-x-2">
                  {subscription.status === 'active' && (
                    <Button
                      variant="outline"
                      size="sm"
                      onClick={() => handleCancelSubscription(subscription.id)}
                    >
                      Cancel
                    </Button>
                  )}
                </div>
              </TableCell>
            </TableRow>
          ))}
        </TableBody>
      </Table>
    </div>
  );
};

export default SubscriptionManagement; 