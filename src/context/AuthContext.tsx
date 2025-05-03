
import React, { createContext, useState, useEffect, useContext } from "react";
import { Session, User } from "@supabase/supabase-js";
import { supabase } from "@/integrations/supabase/client";
import { useToast } from "@/hooks/use-toast";

type AuthContextType = {
  user: User | null;
  session: Session | null;
  loading: boolean;
  signIn: (email: string, password: string) => Promise<void>;
  signUp: (email: string, password: string) => Promise<{ success: boolean; error?: string; confirmationRequired?: boolean }>;
  signOut: () => Promise<void>;
  signInWithGoogle: () => Promise<void>;
  resendVerificationEmail: (email: string) => Promise<{ success: boolean; error?: string }>;
};

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export function AuthProvider({ children }: { children: React.ReactNode }) {
  const [user, setUser] = useState<User | null>(null);
  const [session, setSession] = useState<Session | null>(null);
  const [loading, setLoading] = useState(true);
  const { toast } = useToast();

  useEffect(() => {
    // Set up auth state listener FIRST
    const { data: { subscription } } = supabase.auth.onAuthStateChange(
      (event, session) => {
        console.log("Auth state changed:", event);
        setSession(session);
        setUser(session?.user ?? null);
        setLoading(false);
      }
    );

    // THEN check for existing session
    supabase.auth.getSession().then(({ data: { session } }) => {
      setSession(session);
      setUser(session?.user ?? null);
      setLoading(false);
    });

    return () => subscription.unsubscribe();
  }, []);

  const signIn = async (email: string, password: string) => {
    try {
      const { error } = await supabase.auth.signInWithPassword({
        email,
        password,
      });

      if (error) throw error;
      
      toast({
        title: "Welcome back!",
        description: "You have been successfully logged in.",
      });
    } catch (error: any) {
      toast({
        title: "Error signing in",
        description: error.message,
        variant: "destructive",
      });
      throw error;
    }
  };

  const signUp = async (email: string, password: string) => {
    try {
      const { data, error } = await supabase.auth.signUp({
        email,
        password,
      });

      if (error) throw error;
      
      // Check if email confirmation is required but failed to send
      // This means the account was created but the email wasn't sent
      const confirmationRequired = !data?.session && data?.user;
      
      if (confirmationRequired) {
        toast({
          title: "Account created",
          description: "Please check your email to confirm your account. If you don't receive an email, you can request a new confirmation email.",
        });
      } else if (data?.session) {
        // Auto-sign in successful (email confirmation disabled in Supabase settings)
        toast({
          title: "Account created",
          description: "Your account has been created successfully!",
        });
      }
      
      return { success: true, confirmationRequired };
    } catch (error: any) {
      console.error("Signup error details:", error);
      
      // Special handling for email sending failures
      if (error.message === "Error sending confirmation email") {
        toast({
          title: "Account created",
          description: "Your account was created, but we couldn't send a confirmation email. You can try to resend it later.",
        });
        return { 
          success: true, 
          confirmationRequired: true,
          error: "Error sending confirmation email" 
        };
      }
      
      toast({
        title: "Error signing up",
        description: error.message,
        variant: "destructive",
      });
      
      return { success: false, error: error.message };
    }
  };

  const resendVerificationEmail = async (email: string) => {
    try {
      const { error } = await supabase.auth.resend({
        type: "signup",
        email,
      });

      if (error) throw error;
      
      toast({
        title: "Verification email sent",
        description: "Please check your inbox for the verification email.",
      });
      
      return { success: true };
    } catch (error: any) {
      console.error("Error resending verification email:", error);
      
      toast({
        title: "Error sending email",
        description: error.message,
        variant: "destructive",
      });
      
      return { success: false, error: error.message };
    }
  };

  const signInWithGoogle = async () => {
    try {
      const { error } = await supabase.auth.signInWithOAuth({
        provider: 'google',
        options: {
          redirectTo: window.location.origin,
        }
      });

      if (error) throw error;
      
    } catch (error: any) {
      toast({
        title: "Error signing in with Google",
        description: error.message,
        variant: "destructive",
      });
      throw error;
    }
  };

  const signOut = async () => {
    try {
      // Always clear local state first to ensure UI responsiveness
      setUser(null);
      setSession(null);
      
      try {
        // Then attempt to sign out from Supabase
        const { error } = await supabase.auth.signOut();
        
        if (error) {
          console.error("Error during sign out:", error.message);
          // We don't need to show an error toast here since we already cleared the local state
        }
      } catch (error: any) {
        // Catch any exceptions during sign out but don't show to the user
        // since we already cleared the local state
        console.error("Exception during sign out:", error);
      }
      
      toast({
        title: "Signed out",
        description: "You have been successfully logged out.",
      });
    } catch (error: any) {
      console.error("Exception in signOut function:", error);
      toast({
        title: "Error signing out",
        description: "An unexpected error occurred.",
        variant: "destructive",
      });
    }
  };

  return (
    <AuthContext.Provider
      value={{
        user,
        session,
        loading,
        signIn,
        signUp,
        signOut,
        signInWithGoogle,
        resendVerificationEmail,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
}

export function useAuth() {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error("useAuth must be used within an AuthProvider");
  }
  return context;
}
