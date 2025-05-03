
-- Add an RPC function to enable realtime for tables
CREATE OR REPLACE FUNCTION public.enable_realtime_for_table(table_name text)
RETURNS void AS $$
BEGIN
    EXECUTE format('ALTER TABLE public.%I REPLICA IDENTITY FULL;', table_name);
    EXECUTE format('ALTER PUBLICATION supabase_realtime ADD TABLE public.%I;', table_name);
END;
$$ LANGUAGE plpgsql SECURITY DEFINER;

-- Remove the api_key column from the robots table
-- We're keeping it commented out in case we need to run it manually later
-- ALTER TABLE robots DROP COLUMN IF EXISTS api_key;

-- Enable realtime for the robots table to get telemetry updates
SELECT enable_realtime_for_table('robots');

-- Add telemetry retention days to profiles table
ALTER TABLE profiles ADD COLUMN IF NOT EXISTS telemetry_retention_days INTEGER DEFAULT 7;

-- Update existing profiles to have the default retention period
UPDATE profiles SET telemetry_retention_days = 7 WHERE telemetry_retention_days IS NULL;

-- Add subscription columns to profiles table
ALTER TABLE profiles ADD COLUMN IF NOT EXISTS subscription_plan TEXT;
ALTER TABLE profiles ADD COLUMN IF NOT EXISTS subscription_end TIMESTAMPTZ;

-- Create index on telemetry table to improve query performance for history lookups
CREATE INDEX IF NOT EXISTS idx_telemetry_robot_created_at ON telemetry(robot_id, created_at);
