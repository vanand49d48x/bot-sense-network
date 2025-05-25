   -- Drop the trigger if it exists (for idempotency)
   DROP TRIGGER IF EXISTS on_auth_user_created ON auth.users;

   -- Create the trigger
   CREATE TRIGGER on_auth_user_created
   AFTER INSERT ON auth.users
   FOR EACH ROW EXECUTE FUNCTION public.handle_new_user();