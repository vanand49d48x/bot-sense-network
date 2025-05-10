-- Add admin role column to profiles table
DO $$
BEGIN
  IF EXISTS (
    SELECT FROM information_schema.tables
    WHERE table_name = 'profiles'
  ) THEN
    ALTER TABLE profiles ADD COLUMN IF NOT EXISTS is_admin BOOLEAN DEFAULT false;
  END IF;
END
$$;



-- Create a policy to allow admins to access all data
CREATE POLICY "Admins can access all data" ON profiles
    FOR ALL
    USING (auth.uid() IN (
        SELECT id FROM profiles WHERE is_admin = true
    ));

-- Create a policy to allow admins to access all robots
CREATE POLICY "Admins can access all robots" ON robots
    FOR ALL
    USING (auth.uid() IN (
        SELECT id FROM profiles WHERE is_admin = true
    ));

-- Create a policy to allow admins to access all subscriptions
CREATE POLICY "Admins can access all subscriptions" ON subscriptions
    FOR ALL
    USING (auth.uid() IN (
        SELECT id FROM profiles WHERE is_admin = true
    ));

-- Create a policy to allow admins to access all alerts
CREATE POLICY "Admins can access all alerts" ON alerts
    FOR ALL
    USING (auth.uid() IN (
        SELECT id FROM profiles WHERE is_admin = true
    )); 