
import { createClient } from '@supabase/supabase-js';
import type { Database } from '@/types/supabase';

// Environment configuration
type EnvironmentConfig = {
  url: string;
  key: string;
  isUsingDefaults: boolean;
};

// Get Supabase URL and key from environment variables with fallbacks
const getEnvironmentConfig = (): EnvironmentConfig => {
  // Check for environment variables
  const envUrl = import.meta.env.VITE_SUPABASE_URL;
  const envKey = import.meta.env.VITE_SUPABASE_ANON_KEY;

  // For development in Lovable preview environment, provide default values
  // These are only used in development and should be replaced with proper env vars in production
  const defaultUrl = 'https://uwmbdporlrduzthgdmcg.supabase.co';
  const defaultKey = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InV3bWJkcG9ybHJkdXp0aGdkbWNnIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NDU4NzQ4NzcsImV4cCI6MjA2MTQ1MDg3N30.lkMYbGV9yid6e09hbts9zqwVPZ_DQLW_RNJY_zZ3UWo';

  // Use environment variables if available, otherwise use defaults
  const isUsingDefaults = !envUrl || !envKey;
  
  if (isUsingDefaults) {
    console.warn('⚠️ Using default Supabase credentials. For production, set VITE_SUPABASE_URL and VITE_SUPABASE_ANON_KEY in your environment.');
  }

  return {
    url: envUrl || defaultUrl,
    key: envKey || defaultKey,
    isUsingDefaults
  };
};

// Get the config for current environment
const config = getEnvironmentConfig();

// For logging/debugging purposes
console.log(`Using Supabase environment: ${import.meta.env.MODE === "production" ? "prod" : "dev"}`);
console.log(`Supabase URL: ${config.isUsingDefaults ? 'using default credentials' : 'configured from environment variables'}`);

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
  isUsingEnvVars: !config.isUsingDefaults,
  isUsingDefaults: config.isUsingDefaults
};
