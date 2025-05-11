
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

  // Throw error if environment variables are not set
  if (!envUrl || !envKey) {
    throw new Error(
      'Missing Supabase environment variables. Make sure VITE_SUPABASE_URL and VITE_SUPABASE_ANON_KEY are set.'
    );
  }

  // Return configuration from environment variables
  return {
    url: envUrl,
    key: envKey
  };
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
  isUsingEnvVars: true // All connections now use environment variables
};
