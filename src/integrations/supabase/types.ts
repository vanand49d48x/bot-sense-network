export type Json =
  | string
  | number
  | boolean
  | null
  | { [key: string]: Json | undefined }
  | Json[]

export type Database = {
  public: {
    Tables: {
      admin_users: {
        Row: {
          granted_at: string | null
          granted_by: string | null
          id: string
        }
        Insert: {
          granted_at?: string | null
          granted_by?: string | null
          id: string
        }
        Update: {
          granted_at?: string | null
          granted_by?: string | null
          id?: string
        }
        Relationships: [
          {
            foreignKeyName: "admin_users_id_fkey"
            columns: ["id"]
            isOneToOne: true
            referencedRelation: "user_admin_view"
            referencedColumns: ["admin_user_id"]
          },
        ]
      }
      alerts: {
        Row: {
          created_at: string
          id: string
          message: string
          notification_sent: boolean
          resolved: boolean | null
          resolved_at: string | null
          robot_id: string
          type: string
        }
        Insert: {
          created_at?: string
          id?: string
          message: string
          notification_sent?: boolean
          resolved?: boolean | null
          resolved_at?: string | null
          robot_id: string
          type: string
        }
        Update: {
          created_at?: string
          id?: string
          message?: string
          notification_sent?: boolean
          resolved?: boolean | null
          resolved_at?: string | null
          robot_id?: string
          type?: string
        }
        Relationships: [
          {
            foreignKeyName: "fk_alerts_robotid"
            columns: ["robot_id"]
            isOneToOne: false
            referencedRelation: "robots"
            referencedColumns: ["id"]
          },
        ]
      }
      plan_limits: {
        Row: {
          advanced_analytics: boolean | null
          alerts_per_day: number | null
          api_access: boolean | null
          created_at: string
          custom_telemetry_types: number | null
          id: string
          plan_name: string
          robot_limit: number | null
          support_level: string | null
          telemetry_days: number | null
          updated_at: string
        }
        Insert: {
          advanced_analytics?: boolean | null
          alerts_per_day?: number | null
          api_access?: boolean | null
          created_at?: string
          custom_telemetry_types?: number | null
          id?: string
          plan_name: string
          robot_limit?: number | null
          support_level?: string | null
          telemetry_days?: number | null
          updated_at?: string
        }
        Update: {
          advanced_analytics?: boolean | null
          alerts_per_day?: number | null
          api_access?: boolean | null
          created_at?: string
          custom_telemetry_types?: number | null
          id?: string
          plan_name?: string
          robot_limit?: number | null
          support_level?: string | null
          telemetry_days?: number | null
          updated_at?: string
        }
        Relationships: []
      }
      profiles: {
        Row: {
          api_key: string | null
          avatar_url: string | null
          created_at: string
          custom_alerts: Json[] | null
          custom_robot_types: string[] | null
          custom_telemetry_types: string[] | null
          email: string | null
          first_name: string | null
          id: string
          last_name: string | null
          updated_at: string
        }
        Insert: {
          api_key?: string | null
          avatar_url?: string | null
          created_at?: string
          custom_alerts?: Json[] | null
          custom_robot_types?: string[] | null
          custom_telemetry_types?: string[] | null
          email?: string | null
          first_name?: string | null
          id: string
          last_name?: string | null
          updated_at?: string
        }
        Update: {
          api_key?: string | null
          avatar_url?: string | null
          created_at?: string
          custom_alerts?: Json[] | null
          custom_robot_types?: string[] | null
          custom_telemetry_types?: string[] | null
          email?: string | null
          first_name?: string | null
          id?: string
          last_name?: string | null
          updated_at?: string
        }
        Relationships: [
          {
            foreignKeyName: "profiles_id_fkey"
            columns: ["id"]
            isOneToOne: true
            referencedRelation: "user_admin_view"
            referencedColumns: ["admin_user_id"]
          },
        ]
      }
      robots: {
        Row: {
          api_key: string
          battery_level: number | null
          created_at: string
          description: string | null
          id: string
          last_ping: string | null
          location: Json | null
          name: string
          status: string
          telemetry_data: Json | null
          temperature: number | null
          type: string
          updated_at: string
          user_id: string
        }
        Insert: {
          api_key: string
          battery_level?: number | null
          created_at?: string
          description?: string | null
          id?: string
          last_ping?: string | null
          location?: Json | null
          name: string
          status?: string
          telemetry_data?: Json | null
          temperature?: number | null
          type: string
          updated_at?: string
          user_id: string
        }
        Update: {
          api_key?: string
          battery_level?: number | null
          created_at?: string
          description?: string | null
          id?: string
          last_ping?: string | null
          location?: Json | null
          name?: string
          status?: string
          telemetry_data?: Json | null
          temperature?: number | null
          type?: string
          updated_at?: string
          user_id?: string
        }
        Relationships: [
          {
            foreignKeyName: "fk_robots_user"
            columns: ["user_id"]
            isOneToOne: false
            referencedRelation: "profiles"
            referencedColumns: ["id"]
          },
          {
            foreignKeyName: "robots_user_id_fkey"
            columns: ["user_id"]
            isOneToOne: false
            referencedRelation: "user_admin_view"
            referencedColumns: ["admin_user_id"]
          },
        ]
      }
      subscriptions: {
        Row: {
          created_at: string
          current_period_end: string | null
          id: string
          plan_id: string | null
          plan_name: string | null
          status: string
          stripe_customer_id: string | null
          stripe_subscription_id: string | null
          trial_ended_at: string | null
          trial_started_at: string | null
          trial_status: string | null
          updated_at: string
          user_id: string
        }
        Insert: {
          created_at?: string
          current_period_end?: string | null
          id?: string
          plan_id?: string | null
          plan_name?: string | null
          status?: string
          stripe_customer_id?: string | null
          stripe_subscription_id?: string | null
          trial_ended_at?: string | null
          trial_started_at?: string | null
          trial_status?: string | null
          updated_at?: string
          user_id: string
        }
        Update: {
          created_at?: string
          current_period_end?: string | null
          id?: string
          plan_id?: string | null
          plan_name?: string | null
          status?: string
          stripe_customer_id?: string | null
          stripe_subscription_id?: string | null
          trial_ended_at?: string | null
          trial_started_at?: string | null
          trial_status?: string | null
          updated_at?: string
          user_id?: string
        }
        Relationships: [
          {
            foreignKeyName: "fk_subscriptions_user"
            columns: ["user_id"]
            isOneToOne: true
            referencedRelation: "profiles"
            referencedColumns: ["id"]
          },
          {
            foreignKeyName: "subscriptions_user_id_fkey"
            columns: ["user_id"]
            isOneToOne: true
            referencedRelation: "user_admin_view"
            referencedColumns: ["admin_user_id"]
          },
        ]
      }
      telemetry: {
        Row: {
          battery_level: number | null
          created_at: string
          error_codes: string[] | null
          id: string
          location: Json | null
          motor_status: Json | null
          robot_id: string
          temperature: number | null
        }
        Insert: {
          battery_level?: number | null
          created_at?: string
          error_codes?: string[] | null
          id?: string
          location?: Json | null
          motor_status?: Json | null
          robot_id: string
          temperature?: number | null
        }
        Update: {
          battery_level?: number | null
          created_at?: string
          error_codes?: string[] | null
          id?: string
          location?: Json | null
          motor_status?: Json | null
          robot_id?: string
          temperature?: number | null
        }
        Relationships: []
      }
    }
    Views: {
      user_admin_view: {
        Row: {
          admin_user_id: string | null
          created_at: string | null
          email: string | null
          first_name: string | null
          last_name: string | null
        }
        Relationships: []
      }
    }
    Functions: {
      check_if_admin: {
        Args: { user_id: string }
        Returns: boolean
      }
      create_first_admin: {
        Args: { admin_email: string }
        Returns: undefined
      }
      get_alerts_by_robot: {
        Args: { in_robot_id: string }
        Returns: {
          id: string
          robot_id: string
          type: string
          message: string
          created_at: string
        }[]
      }
      get_plan_limits: {
        Args: Record<PropertyKey, never>
        Returns: {
          id: string
          plan: string
          limit_name: string
          value: number
        }[]
      }
      get_profile_by_id: {
        Args: { profile_id: string }
        Returns: {
          id: string
          first_name: string
          last_name: string
          email: string
        }[]
      }
      get_robots_by_user: {
        Args: { in_user_id: string }
        Returns: {
          id: string
          name: string
          type: string
          user_id: string
        }[]
      }
      get_subscriptions_by_user: {
        Args: { in_user_id: string }
        Returns: {
          id: string
          user_id: string
          status: string
          plan: string
          created_at: string
        }[]
      }
      get_telemetry_by_robot: {
        Args: { in_robot_id: string }
        Returns: {
          id: string
          robot_id: string
          data: Json
          created_at: string
        }[]
      }
      is_admin: {
        Args: { in_uid: string }
        Returns: boolean
      }
    }
    Enums: {
      [_ in never]: never
    }
    CompositeTypes: {
      [_ in never]: never
    }
  }
}

type DefaultSchema = Database[Extract<keyof Database, "public">]

export type Tables<
  DefaultSchemaTableNameOrOptions extends
    | keyof (DefaultSchema["Tables"] & DefaultSchema["Views"])
    | { schema: keyof Database },
  TableName extends DefaultSchemaTableNameOrOptions extends {
    schema: keyof Database
  }
    ? keyof (Database[DefaultSchemaTableNameOrOptions["schema"]]["Tables"] &
        Database[DefaultSchemaTableNameOrOptions["schema"]]["Views"])
    : never = never,
> = DefaultSchemaTableNameOrOptions extends { schema: keyof Database }
  ? (Database[DefaultSchemaTableNameOrOptions["schema"]]["Tables"] &
      Database[DefaultSchemaTableNameOrOptions["schema"]]["Views"])[TableName] extends {
      Row: infer R
    }
    ? R
    : never
  : DefaultSchemaTableNameOrOptions extends keyof (DefaultSchema["Tables"] &
        DefaultSchema["Views"])
    ? (DefaultSchema["Tables"] &
        DefaultSchema["Views"])[DefaultSchemaTableNameOrOptions] extends {
        Row: infer R
      }
      ? R
      : never
    : never

export type TablesInsert<
  DefaultSchemaTableNameOrOptions extends
    | keyof DefaultSchema["Tables"]
    | { schema: keyof Database },
  TableName extends DefaultSchemaTableNameOrOptions extends {
    schema: keyof Database
  }
    ? keyof Database[DefaultSchemaTableNameOrOptions["schema"]]["Tables"]
    : never = never,
> = DefaultSchemaTableNameOrOptions extends { schema: keyof Database }
  ? Database[DefaultSchemaTableNameOrOptions["schema"]]["Tables"][TableName] extends {
      Insert: infer I
    }
    ? I
    : never
  : DefaultSchemaTableNameOrOptions extends keyof DefaultSchema["Tables"]
    ? DefaultSchema["Tables"][DefaultSchemaTableNameOrOptions] extends {
        Insert: infer I
      }
      ? I
      : never
    : never

export type TablesUpdate<
  DefaultSchemaTableNameOrOptions extends
    | keyof DefaultSchema["Tables"]
    | { schema: keyof Database },
  TableName extends DefaultSchemaTableNameOrOptions extends {
    schema: keyof Database
  }
    ? keyof Database[DefaultSchemaTableNameOrOptions["schema"]]["Tables"]
    : never = never,
> = DefaultSchemaTableNameOrOptions extends { schema: keyof Database }
  ? Database[DefaultSchemaTableNameOrOptions["schema"]]["Tables"][TableName] extends {
      Update: infer U
    }
    ? U
    : never
  : DefaultSchemaTableNameOrOptions extends keyof DefaultSchema["Tables"]
    ? DefaultSchema["Tables"][DefaultSchemaTableNameOrOptions] extends {
        Update: infer U
      }
      ? U
      : never
    : never

export type Enums<
  DefaultSchemaEnumNameOrOptions extends
    | keyof DefaultSchema["Enums"]
    | { schema: keyof Database },
  EnumName extends DefaultSchemaEnumNameOrOptions extends {
    schema: keyof Database
  }
    ? keyof Database[DefaultSchemaEnumNameOrOptions["schema"]]["Enums"]
    : never = never,
> = DefaultSchemaEnumNameOrOptions extends { schema: keyof Database }
  ? Database[DefaultSchemaEnumNameOrOptions["schema"]]["Enums"][EnumName]
  : DefaultSchemaEnumNameOrOptions extends keyof DefaultSchema["Enums"]
    ? DefaultSchema["Enums"][DefaultSchemaEnumNameOrOptions]
    : never

export type CompositeTypes<
  PublicCompositeTypeNameOrOptions extends
    | keyof DefaultSchema["CompositeTypes"]
    | { schema: keyof Database },
  CompositeTypeName extends PublicCompositeTypeNameOrOptions extends {
    schema: keyof Database
  }
    ? keyof Database[PublicCompositeTypeNameOrOptions["schema"]]["CompositeTypes"]
    : never = never,
> = PublicCompositeTypeNameOrOptions extends { schema: keyof Database }
  ? Database[PublicCompositeTypeNameOrOptions["schema"]]["CompositeTypes"][CompositeTypeName]
  : PublicCompositeTypeNameOrOptions extends keyof DefaultSchema["CompositeTypes"]
    ? DefaultSchema["CompositeTypes"][PublicCompositeTypeNameOrOptions]
    : never

export const Constants = {
  public: {
    Enums: {},
  },
} as const
