
import React from 'react';
import { supabaseEnv } from '@/integrations/supabase/client';
import { Badge } from '@/components/ui/badge';

export default function EnvironmentIndicator() {
  const { activeEnvironment, isProduction, isUsingEnvVars } = supabaseEnv;
  
  if (!import.meta.env.DEV) return null; // Only show in development
  
  return (
    <Badge 
      className={`fixed bottom-4 right-4 z-50 ${isProduction ? 'bg-red-500' : 'bg-green-500'}`}
    >
      {isProduction ? 'PRODUCTION' : 'DEVELOPMENT'} Environment
      {isUsingEnvVars && ' (Using ENV Variables)'}
    </Badge>
  );
}
