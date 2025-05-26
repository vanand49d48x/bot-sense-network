
import { createClient } from '@supabase/supabase-js';
import type { Database } from '@/types/supabase';

const SUPABASE_URL = "https://rtcspemkxqiecoqeeuai.supabase.co";
const SUPABASE_PUBLISHABLE_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InJ0Y3NwZW1reHFpZWNvcWVldWFpIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NDY4OTk1NjUsImV4cCI6MjA2MjQ3NTU2NX0.mxBM9OzA4kK1XjK5NjoIFLNEylHRLCmGqz6omBPLVJk";

// Import the supabase client like this:
// import { supabase } from "@/integrations/supabase/client";

export const supabase = createClient<Database>(SUPABASE_URL, SUPABASE_PUBLISHABLE_KEY, {
  auth: {
    persistSession: true,
    autoRefreshToken: true,
    storage: localStorage
  }
});
