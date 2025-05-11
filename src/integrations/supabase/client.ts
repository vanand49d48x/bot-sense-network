
import { createClient } from '@supabase/supabase-js';
import type { Database } from '@/types/supabase';

// Environment configuration
type EnvironmentConfig = {
  url: string;
  key: string;
};

// Default Supabase connection values - these will be used if environment variables are not set
const DEFAULT_SUPABASE_URL = "https://uwmbdporlrduzthgdmcg.supabase.co";
const DEFAULT_SUPABASE_ANON_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InV3bWJkcG9ybHJkdXp0aGdkbWNnIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NDU4NzQ4NzcsImV4cCI6MjA2MTQ1MDg3N30.lkMYbGV9yid6e09hbts9zqwVPZ_DQLW_RNJY_zZ3UWo";

// Get Supabase URL and key from environment variables
const getEnvironmentConfig = (): EnvironmentConfig => {
  // Check for environment variables
  const envUrl = import.meta.env.VITE_SUPABASE_URL;
  const envKey = import.meta.env.VITE_SUPABASE_ANON_KEY;

  // Use environment variables if available, otherwise use default values
  const url = envUrl || DEFAULT_SUPABASE_URL;
  const key = envKey || DEFAULT_SUPABASE_ANON_KEY;

  console.log(`Using Supabase URL: ${url.substring(0, 10)}...`);
  
  return { url, key };
};

// Get the config for current environment
const config = getEnvironmentConfig();

// For logging/debugging purposes
console.log(`Using Supabase environment: ${import.meta.env.MODE === "production" ? "prod" : "dev"}`);

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
  isUsingEnvVars: !!(import.meta.env.VITE_SUPABASE_URL && import.meta.env.VITE_SUPABASE_ANON_KEY)
};
