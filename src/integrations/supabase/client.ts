
import { createClient } from '@supabase/supabase-js';
import type { Database } from '@/types/supabase';

// Environment-specific URLs and keys
const ENVIRONMENTS = {
  dev: {
    url: "https://rtcspemkxqiecoqeeuai.supabase.co",
    key: "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InJ0Y3NwZW1reHFpZWNvcWVldWFpIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NDY4OTk1NjUsImV4cCI6MjA2MjQ3NTU2NX0.mxBM9OzA4kK1XjK5NjoIFLNEylHRLCmGqz6omBPLVJk"
  },
  prod: {
    url: "https://uwmbdporlrduzthgdmcg.supabase.co",
    key: "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InV3bWJkcG9ybHJkdXp0aGdkbWNnIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NDU4NzQ4NzcsImV4cCI6MjA2MTQ1MDg3N30.lkMYbGV9yid6e09hbts9zqwVPZ_DQLW_RNJY_zZ3UWo"
  }
};

// Determine the active environment
// Use "prod" for production builds, "dev" otherwise
// This can be configured via build process or environment variables
const ACTIVE_ENVIRONMENT = import.meta.env.MODE === "production" ? "prod" : "dev";

// Get the appropriate URL and key for the active environment
const SUPABASE_URL = ENVIRONMENTS[ACTIVE_ENVIRONMENT].url;
const SUPABASE_PUBLISHABLE_KEY = ENVIRONMENTS[ACTIVE_ENVIRONMENT].key;

// For logging/debugging purposes
console.log(`Using Supabase environment: ${ACTIVE_ENVIRONMENT}`);

// Import the supabase client like this:
// import { supabase } from "@/integrations/supabase/client";

export const supabase = createClient<Database>(SUPABASE_URL, SUPABASE_PUBLISHABLE_KEY, {
  auth: {
    persistSession: true,
    autoRefreshToken: true,
    storage: localStorage
  }
});

// Export environment info to allow components to know which environment is active
export const supabaseEnv = {
  activeEnvironment: ACTIVE_ENVIRONMENT,
  isProduction: ACTIVE_ENVIRONMENT === "prod",
  isDevelopment: ACTIVE_ENVIRONMENT === "dev"
};
