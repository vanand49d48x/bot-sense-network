-- Grant INSERT on profiles to authenticated and service_role
GRANT INSERT ON TABLE public.profiles TO authenticated;
GRANT INSERT ON TABLE public.profiles TO service_role;

-- Allow service_role to insert any profile (for triggers/functions)
CREATE POLICY "Service role can insert profiles"
ON public.profiles
FOR INSERT
TO service_role
WITH CHECK (true);

-- Allow authenticated users to insert their own profile (optional, but recommended)
CREATE POLICY "Authenticated can insert own profile"
ON public.profiles
FOR INSERT
TO authenticated
WITH CHECK (auth.uid() = id);

-- (Optional) Make sure your trigger function is SECURITY DEFINER
CREATE OR REPLACE FUNCTION public.handle_new_user()
RETURNS trigger AS $$
BEGIN
  INSERT INTO public.profiles (id, first_name, last_name)
  VALUES (NEW.id, '', '');
  RETURN NEW;
END;
$$ LANGUAGE plpgsql SECURITY DEFINER;