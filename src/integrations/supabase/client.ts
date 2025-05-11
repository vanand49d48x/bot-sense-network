
import { createClient } from '@supabase/supabase-js';
import type { Database } from '@/types/supabase';

// Environment configuration
type EnvironmentConfig = {
  url: string;
  key: string;
};

// Get Supabase URL and key from environment variables
const getEnvironmentConfig = (): EnvironmentConfig => {
  // Check for environment variables
  const envUrl = import.meta.env.VITE_SUPABASE_URL;
  const envKey = import.meta.env.VITE_SUPABASE_ANON_KEY;

  // Determine environment based on build mode
  const ACTIVE_ENVIRONMENT = import.meta.env.MODE === "production" ? "prod" : "dev";
  
  // Fallback values for local development only
  // These will be used only if environment variables are not set
  const FALLBACK_ENVIRONMENTS = {
    dev: {
      url: "https://rtcspemkxqiecoqeeuai.supabase.co",
      key: "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InJ0Y3NwZW1reHFpZWNvcWVldWFpIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NDY4OTk1NjUsImV4cCI6MjA2MjQ3NTU2NX0.mxBM9OzA4kK1XjK5NjoIFLNEylHRLCmGqz6omBPLVJk"
    },
    prod: {
      url: "https://uwmbdporlrduzthgdmcg.supabase.co",
      key: "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InV3bWJkcG9ybHJkdXp0aGdkbWNnIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NDU4NzQ4NzcsImV4cCI6MjA2MTQ1MDg3N30.lkMYbGV9yid6e09hbts9zqwVPZ_DQLW_RNJY_zZ3UWo"
    }
  };

  // Priority: Use environment variables if available, otherwise use fallbacks
  return {
    url: envUrl || FALLBACK_ENVIRONMENTS[ACTIVE_ENVIRONMENT].url,
    key: envKey || FALLBACK_ENVIRONMENTS[ACTIVE_ENVIRONMENT].key
  };
};

// Get the config for current environment
const config = getEnvironmentConfig();

// For logging/debugging purposes
console.log(`Using Supabase environment: ${import.meta.env.MODE === "production" ? "prod" : "dev"}`);
console.log(`Using Supabase URL from: ${import.meta.env.VITE_SUPABASE_URL ? 'environment variable' : 'fallback'}`);

// Import the supabase client like this:
// import { supabase } from "@/integrations/supabase/client";

export const supabase = createClient<Database>(config.url, config.key, {
  auth: {
    persistSession: true,
    autoRefreshToken: true,
    storage: localStorage
  }
});

// Export environment info to allow components to know which environment is active
export const supabaseEnv = {
  activeEnvironment: import.meta.env.MODE === "production" ? "prod" : "dev",
  isProduction: import.meta.env.MODE === "production",
  isDevelopment: import.meta.env.MODE !== "production",
  isUsingEnvVars: !!import.meta.env.VITE_SUPABASE_URL
};
