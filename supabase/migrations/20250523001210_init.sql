DROP TRIGGER IF EXISTS trigger_auto_start_free_trial ON auth.users;

CREATE TRIGGER trigger_auto_start_free_trial
AFTER INSERT ON auth.users
FOR EACH ROW EXECUTE FUNCTION public.auto_start_free_trial();