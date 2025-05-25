GRANT INSERT ON TABLE public.subscriptions TO authenticated;
GRANT INSERT ON TABLE public.subscriptions TO service_role;

CREATE POLICY "Authenticated can insert subscriptions"
ON public.subscriptions
FOR INSERT
TO authenticated
WITH CHECK (true);

CREATE OR REPLACE FUNCTION public.auto_start_free_trial()
RETURNS trigger AS $$
DECLARE
  trial_end_date TIMESTAMP WITH TIME ZONE;
BEGIN
  trial_end_date := now() + interval '7 days';
  INSERT INTO public.subscriptions (
    user_id, 
    status, 
    plan_name,
    trial_status, 
    trial_started_at,
    trial_ended_at,
    current_period_end
  ) VALUES (
    NEW.id,
    'active', 
    'Free Tier',
    'active',
    now(),
    trial_end_date,
    trial_end_date
  );
  RETURN NEW;
END;
$$ LANGUAGE plpgsql SECURITY DEFINER;