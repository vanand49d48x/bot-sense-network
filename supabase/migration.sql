
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
