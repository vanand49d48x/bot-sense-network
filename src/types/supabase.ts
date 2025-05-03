import { Json } from "./utils";

export interface Organization {
  id: string;
  created_at?: string;
  name?: string;
  slug?: string;
  logo_url?: string;
  owner_id?: string;
}

export interface Robot {
  id?: string;
  created_at?: string;
  updated_at?: string;
  name?: string;
  slug?: string;
  description?: string;
  image_url?: string;
  owner_id?: string;
  organization_id?: string;
  type?: string;
  status?: string;
  location?: Json;
  telemetry?: Json;
  last_active?: string;
  api_key?: string;
}

export interface Alert {
  id?: string;
  created_at?: string;
  updated_at?: string;
  robot_id?: string;
  type?: string;
  message?: string;
  severity?: string;
  status?: string;
  resolved_at?: string;
  telemetry?: Json;
}

export interface TelemetryHistory {
  id?: string;
  created_at?: string;
  robot_id?: string;
  telemetry?: Json;
}

export interface UserProfile {
  id?: string;
  created_at?: string;
  updated_at?: string;
  first_name?: string;
  last_name?: string;
  avatar_url?: string;
  api_key?: string;
  custom_robot_types?: string[];
  custom_telemetry_types?: string[];
  custom_alerts?: Json[];
  telemetry_retention_days?: number;
}
