
import React from 'react';
import { supabaseEnv } from '@/integrations/supabase/client';
import { Badge } from '@/components/ui/badge';

export default function EnvironmentIndicator() {
  const { activeEnvironment, isProduction, isUsingDefaults } = supabaseEnv;
  
  // Only show in development environment or when using defaults in any environment
  const shouldShow = import.meta.env.DEV || isUsingDefaults;
  
  if (!shouldShow) return null;
  
  return (
    <div className="fixed bottom-4 right-4 z-50 flex flex-col gap-2">
      <Badge 
        className={`${isProduction ? 'bg-red-500' : 'bg-green-500'}`}
      >
        {isProduction ? 'PRODUCTION' : 'DEVELOPMENT'} Environment
      </Badge>
      
      {isUsingDefaults && (
        <Badge className="bg-yellow-500 hover:bg-yellow-400">
          Using Default Supabase Credentials
        </Badge>
      )}
    </div>
  );
}
