
import React from 'react';
import { supabaseEnv } from '@/integrations/supabase/client';
import { Badge } from '@/components/ui/badge';

export default function EnvironmentIndicator() {
  const { activeEnvironment, isProduction } = supabaseEnv;
  
  if (!import.meta.env.DEV) return null; // Only show in development
  
  return (
    <div className="fixed bottom-4 right-4 z-50 flex flex-col gap-2">
      <Badge 
        className={`${isProduction ? 'bg-red-500' : 'bg-green-500'}`}
      >
        {isProduction ? 'PRODUCTION' : 'DEVELOPMENT'} Environment
      </Badge>
    </div>
  );
}
