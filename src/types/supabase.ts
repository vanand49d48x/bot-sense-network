
export type Json =
  | string
  | number
  | boolean
  | null
  | { [key: string]: Json | undefined }
  | Json[]

export interface Database {
  public: {
    Tables: {
      alerts: {
        Row: {
          id: string
          robot_id: string
          type: string
          message: string
          resolved: boolean
          created_at: string
          resolved_at: string | null
        }
        Insert: {
          id?: string
          robot_id: string
          type: string
          message: string
          resolved?: boolean
          created_at?: string
          resolved_at?: string | null
        }
        Update: {
          id?: string
          robot_id?: string
          type?: string
          message?: string
          resolved?: boolean
          created_at?: string
          resolved_at?: string | null
        }
      }
      profiles: {
        Row: {
          id: string
          first_name: string | null
          last_name: string | null
          avatar_url: string | null
          created_at: string
          updated_at: string
        }
        Insert: {
          id: string
          first_name?: string | null
          last_name?: string | null
          avatar_url?: string | null
          created_at?: string
          updated_at?: string
        }
        Update: {
          id?: string
          first_name?: string | null
          last_name?: string | null
          avatar_url?: string | null
          created_at?: string
          updated_at?: string
        }
      }
      robots: {
        Row: {
          id: string
          name: string
          type: string
          description: string | null
          status: string
          last_ping: string
          created_at: string
          updated_at: string
          user_id: string
          battery_level: number
          temperature: number
          location: Json
        }
        Insert: {
          id?: string
          name: string
          type: string
          description?: string | null
          status?: string
          last_ping?: string
          created_at?: string
          updated_at?: string
          user_id: string
          battery_level?: number
          temperature?: number
          location?: Json
        }
        Update: {
          id?: string
          name?: string
          type?: string
          description?: string | null
          status?: string
          last_ping?: string
          created_at?: string
          updated_at?: string
          user_id?: string
          battery_level?: number
          temperature?: number
          location?: Json
        }
      }
      telemetry: {
        Row: {
          id: string
          robot_id: string
          battery_level: number | null
          temperature: number | null
          location: Json | null
          motor_status: Json | null
          error_codes: string[] | null
          created_at: string
        }
        Insert: {
          id?: string
          robot_id: string
          battery_level?: number | null
          temperature?: number | null
          location?: Json | null
          motor_status?: Json | null
          error_codes?: string[] | null
          created_at?: string
        }
        Update: {
          id?: string
          robot_id?: string
          battery_level?: number | null
          temperature?: number | null
          location?: Json | null
          motor_status?: Json | null
          error_codes?: string[] | null
          created_at?: string
        }
      }
    }
  }
}
