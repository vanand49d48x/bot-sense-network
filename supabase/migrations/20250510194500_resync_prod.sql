--
-- PostgreSQL database dump
--

-- Dumped from database version 15.8
-- Dumped by pg_dump version 15.13 (Homebrew)

SET statement_timeout = 0;
SET lock_timeout = 0;
SET idle_in_transaction_session_timeout = 0;
SET client_encoding = 'UTF8';
SET standard_conforming_strings = on;
SELECT pg_catalog.set_config('search_path', '', false);
SET check_function_bodies = false;
SET xmloption = content;
SET client_min_messages = warning;
SET row_security = off;

--
-- Name: public; Type: SCHEMA; Schema: -; Owner: pg_database_owner
--




ALTER SCHEMA public OWNER TO pg_database_owner;

--
-- Name: SCHEMA public; Type: COMMENT; Schema: -; Owner: pg_database_owner
--

COMMENT ON SCHEMA public IS 'standard public schema';


--
-- Name: auto_start_free_trial(); Type: FUNCTION; Schema: public; Owner: postgres
--

CREATE OR REPLACE FUNCTION public.auto_start_free_trial() RETURNS trigger
    LANGUAGE plpgsql
    SET search_path TO 'public'
    AS $$
DECLARE
  trial_end_date TIMESTAMP WITH TIME ZONE;
BEGIN
  trial_end_date := now() + interval '7 days';
  INSERT INTO public.subscriptions (
    user_id, 
    status, 
    plan_name,
    trial_status, 
    trial_started_at,
    trial_ended_at,
    current_period_end
  ) VALUES (
    NEW.id,
    'active', 
    'Free Tier',
    'active',
    now(),
    trial_end_date,
    trial_end_date
  );
  RETURN NEW;
END;
$$;


ALTER FUNCTION public.auto_start_free_trial() OWNER TO postgres;

--
-- Name: check_if_admin(uuid); Type: FUNCTION; Schema: public; Owner: postgres
--

CREATE OR REPLACE FUNCTION public.check_if_admin(user_id uuid) RETURNS boolean
    LANGUAGE plpgsql SECURITY DEFINER
    SET search_path TO 'public'
    AS $$
BEGIN
  RETURN EXISTS (
    SELECT 1
    FROM public.admin_users
    WHERE id = user_id
  );
END;
$$;


ALTER FUNCTION public.check_if_admin(user_id uuid) OWNER TO postgres;

--
-- Name: create_first_admin(text); Type: FUNCTION; Schema: public; Owner: postgres
--

CREATE OR REPLACE FUNCTION public.create_first_admin(admin_email text) RETURNS void
    LANGUAGE plpgsql SECURITY DEFINER
    SET search_path TO 'public'
    AS $$
DECLARE
  admin_id UUID;
BEGIN
  SELECT id INTO admin_id
  FROM auth.users
  WHERE email = admin_email;

  IF admin_id IS NOT NULL THEN
    INSERT INTO public.admin_users (id, granted_by)
    VALUES (admin_id, 'Initial Setup')
    ON CONFLICT (id) DO NOTHING;
  END IF;
END;
$$;


ALTER FUNCTION public.create_first_admin(admin_email text) OWNER TO postgres;

--
-- Name: get_alerts_by_robot(uuid); Type: FUNCTION; Schema: public; Owner: postgres
--

CREATE OR REPLACE FUNCTION public.get_alerts_by_robot(in_robot_id uuid) RETURNS TABLE(id uuid, robot_id uuid, type text, message text, created_at timestamp with time zone)
    LANGUAGE plpgsql
    SET search_path TO 'public'
    AS $$
BEGIN
  RETURN QUERY
  SELECT id, robot_id, type, message, created_at
  FROM public.alerts
  WHERE robot_id = in_robot_id;
END;
$$;


ALTER FUNCTION public.get_alerts_by_robot(in_robot_id uuid) OWNER TO postgres;

--
-- Name: get_plan_limits(); Type: FUNCTION; Schema: public; Owner: postgres
--

CREATE OR REPLACE FUNCTION public.get_plan_limits() RETURNS TABLE(id uuid, plan text, limit_name text, value integer)
    LANGUAGE plpgsql
    SET search_path TO 'public'
    AS $$
BEGIN
  RETURN QUERY
  SELECT id, plan, limit_name, value
  FROM public.plan_limits;
END;
$$;


ALTER FUNCTION public.get_plan_limits() OWNER TO postgres;

--
-- Name: get_profile_by_id(uuid); Type: FUNCTION; Schema: public; Owner: postgres
--

CREATE OR REPLACE FUNCTION public.get_profile_by_id(profile_id uuid) RETURNS TABLE(id uuid, first_name text, last_name text, email text)
    LANGUAGE plpgsql
    SET search_path TO 'public'
    AS $$
BEGIN
  RETURN QUERY
  SELECT id, first_name, last_name, email
  FROM public.profiles
  WHERE id = profile_id;
END;
$$;


ALTER FUNCTION public.get_profile_by_id(profile_id uuid) OWNER TO postgres;

--
-- Name: get_robots_by_user(uuid); Type: FUNCTION; Schema: public; Owner: postgres
--

CREATE OR REPLACE FUNCTION public.get_robots_by_user(in_user_id uuid) RETURNS TABLE(id uuid, name text, type text, user_id uuid)
    LANGUAGE plpgsql
    SET search_path TO 'public'
    AS $$
BEGIN
  RETURN QUERY
  SELECT id, name, type, user_id
  FROM public.robots
  WHERE user_id = in_user_id;
END;
$$;


ALTER FUNCTION public.get_robots_by_user(in_user_id uuid) OWNER TO postgres;

--
-- Name: get_subscriptions_by_user(uuid); Type: FUNCTION; Schema: public; Owner: postgres
--

CREATE OR REPLACE FUNCTION public.get_subscriptions_by_user(in_user_id uuid) RETURNS TABLE(id uuid, user_id uuid, status text, plan text, created_at timestamp with time zone)
    LANGUAGE plpgsql
    SET search_path TO 'public'
    AS $$
BEGIN
  RETURN QUERY
  SELECT id, user_id, status, plan, created_at
  FROM public.subscriptions
  WHERE user_id = in_user_id;
END;
$$;


ALTER FUNCTION public.get_subscriptions_by_user(in_user_id uuid) OWNER TO postgres;

--
-- Name: get_telemetry_by_robot(uuid); Type: FUNCTION; Schema: public; Owner: postgres
--

CREATE OR REPLACE FUNCTION public.get_telemetry_by_robot(in_robot_id uuid) RETURNS TABLE(id uuid, robot_id uuid, data jsonb, created_at timestamp with time zone)
    LANGUAGE plpgsql
    SET search_path TO 'public'
    AS $$
BEGIN
  RETURN QUERY
  SELECT id, robot_id, data, created_at
  FROM public.telemetry
  WHERE robot_id = in_robot_id;
END;
$$;


ALTER FUNCTION public.get_telemetry_by_robot(in_robot_id uuid) OWNER TO postgres;

--
-- Name: handle_new_user(); Type: FUNCTION; Schema: public; Owner: postgres
--

CREATE OR REPLACE FUNCTION public.handle_new_user() RETURNS trigger
    LANGUAGE plpgsql
    SET search_path TO 'public'
    AS $$
BEGIN
  INSERT INTO public.profiles (id, first_name, last_name)
  VALUES (NEW.id, '', '');
  RETURN NEW;
END;
$$;


ALTER FUNCTION public.handle_new_user() OWNER TO postgres;

--
-- Name: is_admin(uuid); Type: FUNCTION; Schema: public; Owner: postgres
--

CREATE OR REPLACE FUNCTION public.is_admin(in_uid uuid) RETURNS boolean
    LANGUAGE plpgsql
    SET search_path TO 'public'
    AS $$
BEGIN
  RETURN EXISTS (SELECT 1 FROM public.admin_users WHERE id = in_uid);
END;
$$;


ALTER FUNCTION public.is_admin(in_uid uuid) OWNER TO postgres;

--
-- Name: update_robot_status(); Type: FUNCTION; Schema: public; Owner: postgres
--

CREATE OR REPLACE FUNCTION public.update_robot_status() RETURNS trigger
    LANGUAGE plpgsql
    SET search_path TO 'public'
    AS $$
BEGIN
  UPDATE public.robots
  SET 
    battery_level = COALESCE(NEW.battery_level, battery_level),
    temperature = COALESCE(NEW.temperature, temperature),
    location = COALESCE(NEW.location, location),
    last_ping = NOW(),
    status = 'online',
    updated_at = NOW()
  WHERE id = NEW.robot_id;
  RETURN NEW;
END;
$$;


ALTER FUNCTION public.update_robot_status() OWNER TO postgres;

--
-- Name: update_updated_at_column(); Type: FUNCTION; Schema: public; Owner: postgres
--

CREATE OR REPLACE FUNCTION public.update_updated_at_column() RETURNS trigger
    LANGUAGE plpgsql
    SET search_path TO 'public'
    AS $$
BEGIN
    NEW.updated_at = now();
    RETURN NEW;
END;
$$;


ALTER FUNCTION public.update_updated_at_column() OWNER TO postgres;

SET default_tablespace = '';

SET default_table_access_method = heap;

--
-- Name: admin_users; Type: TABLE; Schema: public; Owner: postgres
--

CREATE  TABLE IF NOT EXISTS public.admin_users (
    id uuid NOT NULL,
    granted_at timestamp with time zone DEFAULT now(),
    granted_by text
);


ALTER TABLE public.admin_users OWNER TO postgres;

--
-- Name: alerts; Type: TABLE; Schema: public; Owner: postgres
--

CREATE  TABLE IF NOT EXISTS public.alerts (
    id uuid DEFAULT gen_random_uuid() NOT NULL,
    robot_id uuid NOT NULL,
    type text NOT NULL,
    message text NOT NULL,
    resolved boolean DEFAULT false,
    created_at timestamp with time zone DEFAULT now() NOT NULL,
    resolved_at timestamp with time zone,
    notification_sent boolean DEFAULT false NOT NULL
);

ALTER TABLE ONLY public.alerts REPLICA IDENTITY FULL;


ALTER TABLE public.alerts OWNER TO postgres;

--
-- Name: plan_limits; Type: TABLE; Schema: public; Owner: postgres
--

CREATE  TABLE IF NOT EXISTS public.plan_limits (
    id uuid DEFAULT gen_random_uuid() NOT NULL,
    plan_name text NOT NULL,
    robot_limit integer,
    telemetry_days integer,
    custom_telemetry_types integer,
    alerts_per_day integer,
    support_level text,
    api_access boolean,
    advanced_analytics boolean,
    created_at timestamp with time zone DEFAULT now() NOT NULL,
    updated_at timestamp with time zone DEFAULT now() NOT NULL
);


ALTER TABLE public.plan_limits OWNER TO postgres;

--
-- Name: TABLE plan_limits; Type: COMMENT; Schema: public; Owner: postgres
--

COMMENT ON TABLE public.plan_limits IS 'Configurable limits for subscription plans. NULL values for Enterprise plan represent unlimited resources.';


--
-- Name: profiles; Type: TABLE; Schema: public; Owner: postgres
--

CREATE  TABLE IF NOT EXISTS public.profiles (
    id uuid NOT NULL,
    first_name text,
    last_name text,
    avatar_url text,
    created_at timestamp with time zone DEFAULT now() NOT NULL,
    updated_at timestamp with time zone DEFAULT now() NOT NULL,
    api_key text,
    custom_robot_types text[] DEFAULT '{}'::text[],
    custom_alerts jsonb[],
    custom_telemetry_types text[] DEFAULT '{}'::text[],
    email text
);


ALTER TABLE public.profiles OWNER TO postgres;

--
-- Name: robots; Type: TABLE; Schema: public; Owner: postgres
--

CREATE  TABLE IF NOT EXISTS public.robots (
    id uuid DEFAULT gen_random_uuid() NOT NULL,
    name text NOT NULL,
    type text NOT NULL,
    description text,
    status text DEFAULT 'offline'::text NOT NULL,
    last_ping timestamp with time zone DEFAULT now(),
    created_at timestamp with time zone DEFAULT now() NOT NULL,
    updated_at timestamp with time zone DEFAULT now() NOT NULL,
    user_id uuid NOT NULL,
    battery_level integer DEFAULT 100,
    temperature numeric DEFAULT 25.0,
    location jsonb DEFAULT '{"latitude": 0, "longitude": 0}'::jsonb,
    api_key text NOT NULL,
    telemetry_data jsonb DEFAULT '{}'::jsonb
);

ALTER TABLE ONLY public.robots REPLICA IDENTITY FULL;


ALTER TABLE public.robots OWNER TO postgres;

--
-- Name: subscriptions; Type: TABLE; Schema: public; Owner: postgres
--

CREATE  TABLE IF NOT EXISTS public.subscriptions (
    id uuid DEFAULT gen_random_uuid() NOT NULL,
    user_id uuid NOT NULL,
    stripe_customer_id text,
    stripe_subscription_id text,
    plan_id text,
    plan_name text,
    status text DEFAULT 'inactive'::text NOT NULL,
    current_period_end timestamp with time zone,
    created_at timestamp with time zone DEFAULT now() NOT NULL,
    updated_at timestamp with time zone DEFAULT now() NOT NULL,
    trial_started_at timestamp with time zone,
    trial_ended_at timestamp with time zone,
    trial_status text,
    CONSTRAINT subscriptions_trial_status_check CHECK ((trial_status = ANY (ARRAY['active'::text, 'expired'::text, 'converted'::text])))
);


ALTER TABLE public.subscriptions OWNER TO postgres;

--
-- Name: telemetry; Type: TABLE; Schema: public; Owner: postgres
--

CREATE  TABLE IF NOT EXISTS public.telemetry (
    id uuid DEFAULT gen_random_uuid() NOT NULL,
    robot_id uuid NOT NULL,
    battery_level integer,
    temperature numeric,
    location jsonb,
    motor_status jsonb,
    error_codes text[],
    created_at timestamp with time zone DEFAULT now() NOT NULL
);

ALTER TABLE ONLY public.telemetry REPLICA IDENTITY FULL;


ALTER TABLE public.telemetry OWNER TO postgres;

--
-- Name: user_admin_view; Type: VIEW; Schema: public; Owner: postgres
--

CREATE VIEW public.user_admin_view AS
 SELECT u.id AS admin_user_id,
    u.email,
    p.first_name,
    p.last_name,
    u.created_at
   FROM (auth.users u
     LEFT JOIN public.profiles p ON ((u.id = p.id)))
  ORDER BY u.created_at DESC;


ALTER TABLE public.user_admin_view OWNER TO postgres;

--
-- Name: admin_users admin_users_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.admin_users
    ADD CONSTRAINT admin_users_pkey PRIMARY KEY (id);


--
-- Name: alerts alerts_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.alerts
    ADD CONSTRAINT alerts_pkey PRIMARY KEY (id);


--
-- Name: plan_limits plan_limits_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.plan_limits
    ADD CONSTRAINT plan_limits_pkey PRIMARY KEY (id);


--
-- Name: plan_limits plan_limits_plan_name_key; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.plan_limits
    ADD CONSTRAINT plan_limits_plan_name_key UNIQUE (plan_name);


--
-- Name: profiles profiles_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.profiles
    ADD CONSTRAINT profiles_pkey PRIMARY KEY (id);


--
-- Name: robots robots_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.robots
    ADD CONSTRAINT robots_pkey PRIMARY KEY (id);


--
-- Name: subscriptions subscriptions_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.subscriptions
    ADD CONSTRAINT subscriptions_pkey PRIMARY KEY (id);


--
-- Name: subscriptions subscriptions_user_id_unique; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.subscriptions
    ADD CONSTRAINT subscriptions_user_id_unique UNIQUE (user_id);


--
-- Name: telemetry telemetry_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.telemetry
    ADD CONSTRAINT telemetry_pkey PRIMARY KEY (id);


--
-- Name: idx_subscriptions_stripe_customer_id; Type: INDEX; Schema: public; Owner: postgres
--

CREATE INDEX idx_subscriptions_stripe_customer_id ON public.subscriptions USING btree (stripe_customer_id);


--
-- Name: idx_subscriptions_trial_status; Type: INDEX; Schema: public; Owner: postgres
--

CREATE INDEX idx_subscriptions_trial_status ON public.subscriptions USING btree (trial_status);


--
-- Name: idx_subscriptions_user_id; Type: INDEX; Schema: public; Owner: postgres
--

CREATE INDEX idx_subscriptions_user_id ON public.subscriptions USING btree (user_id);


--
-- Name: telemetry on_telemetry_received; Type: TRIGGER; Schema: public; Owner: postgres
--

CREATE TRIGGER on_telemetry_received AFTER INSERT ON public.telemetry FOR EACH ROW EXECUTE FUNCTION public.update_robot_status();


--
-- Name: plan_limits update_plan_limits_updated_at; Type: TRIGGER; Schema: public; Owner: postgres
--

CREATE TRIGGER update_plan_limits_updated_at BEFORE UPDATE ON public.plan_limits FOR EACH ROW EXECUTE FUNCTION public.update_updated_at_column();


--
-- Name: admin_users admin_users_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.admin_users
    ADD CONSTRAINT admin_users_id_fkey FOREIGN KEY (id) REFERENCES auth.users(id) ON DELETE CASCADE;


--
-- Name: alerts fk_alerts_robotid; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.alerts
    ADD CONSTRAINT fk_alerts_robotid FOREIGN KEY (robot_id) REFERENCES public.robots(id);


--
-- Name: robots fk_robots_user; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.robots
    ADD CONSTRAINT fk_robots_user FOREIGN KEY (user_id) REFERENCES public.profiles(id);


--
-- Name: subscriptions fk_subscriptions_user; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.subscriptions
    ADD CONSTRAINT fk_subscriptions_user FOREIGN KEY (user_id) REFERENCES public.profiles(id);


--
-- Name: profiles profiles_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.profiles
    ADD CONSTRAINT profiles_id_fkey FOREIGN KEY (id) REFERENCES auth.users(id) ON DELETE CASCADE;


--
-- Name: robots robots_user_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.robots
    ADD CONSTRAINT robots_user_id_fkey FOREIGN KEY (user_id) REFERENCES auth.users(id) ON DELETE CASCADE;


--
-- Name: subscriptions subscriptions_user_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.subscriptions
    ADD CONSTRAINT subscriptions_user_id_fkey FOREIGN KEY (user_id) REFERENCES auth.users(id);


--
-- Name: admin_users Admins can add new admin users; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Admins can add new admin users" ON public.admin_users FOR INSERT WITH CHECK (public.check_if_admin(auth.uid()));


--
-- Name: robots Admins can manage all robots; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Admins can manage all robots" ON public.robots USING ((EXISTS ( SELECT 1
   FROM public.admin_users
  WHERE (admin_users.id = auth.uid()))));


--
-- Name: profiles Admins can update all profiles; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Admins can update all profiles" ON public.profiles FOR UPDATE USING ((EXISTS ( SELECT 1
   FROM public.admin_users
  WHERE (admin_users.id = auth.uid()))));


--
-- Name: alerts Admins can view all alerts; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Admins can view all alerts" ON public.alerts FOR SELECT USING ((EXISTS ( SELECT 1
   FROM public.admin_users
  WHERE (admin_users.id = auth.uid()))));


--
-- Name: profiles Admins can view all profiles; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Admins can view all profiles" ON public.profiles FOR SELECT USING ((EXISTS ( SELECT 1
   FROM public.admin_users
  WHERE (admin_users.id = auth.uid()))));


--
-- Name: subscriptions Admins can view all subscriptions; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Admins can view all subscriptions" ON public.subscriptions FOR SELECT USING ((EXISTS ( SELECT 1
   FROM public.admin_users
  WHERE (admin_users.id = auth.uid()))));


--
-- Name: telemetry Admins can view all telemetry; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Admins can view all telemetry" ON public.telemetry FOR SELECT USING ((EXISTS ( SELECT 1
   FROM public.admin_users
  WHERE (admin_users.id = auth.uid()))));


--
-- Name: admin_users Admins can view their own admin_users row; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Admins can view their own admin_users row" ON public.admin_users FOR SELECT USING ((id = auth.uid()));


--
-- Name: admin_users Allow all authenticated users to read admin_users; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Allow all authenticated users to read admin_users" ON public.admin_users FOR SELECT USING ((auth.role() = 'authenticated'::text));


--
-- Name: plan_limits Anyone can read plan limits; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Anyone can read plan limits" ON public.plan_limits FOR SELECT USING (true);


--
-- Name: subscriptions Service role can insert subscriptions; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Service role can insert subscriptions" ON public.subscriptions FOR INSERT WITH CHECK (true);


--
-- Name: subscriptions Service role can update subscriptions; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Service role can update subscriptions" ON public.subscriptions FOR UPDATE USING (true);


--
-- Name: profiles Users and admins can select profiles; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Users and admins can select profiles" ON public.profiles FOR SELECT USING (((auth.uid() = id) OR (EXISTS ( SELECT 1
   FROM public.admin_users
  WHERE (admin_users.id = auth.uid())))));


--
-- Name: robots Users can create their own robots; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Users can create their own robots" ON public.robots FOR INSERT WITH CHECK ((auth.uid() = user_id));


--
-- Name: robots Users can delete their own robots; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Users can delete their own robots" ON public.robots FOR DELETE USING ((auth.uid() = user_id));


--
-- Name: telemetry Users can insert telemetry for their own robots; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Users can insert telemetry for their own robots" ON public.telemetry FOR INSERT WITH CHECK ((( SELECT robots.user_id
   FROM public.robots
  WHERE (robots.id = telemetry.robot_id)) = auth.uid()));


--
-- Name: robots Users can manage their own robots; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Users can manage their own robots" ON public.robots USING ((auth.uid() = user_id));


--
-- Name: profiles Users can select their own profile; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Users can select their own profile" ON public.profiles FOR SELECT USING ((auth.uid() = id));


--
-- Name: alerts Users can update alerts for their own robots; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Users can update alerts for their own robots" ON public.alerts FOR UPDATE USING ((( SELECT robots.user_id
   FROM public.robots
  WHERE (robots.id = alerts.robot_id)) = auth.uid()));


--
-- Name: profiles Users can update their own profile; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Users can update their own profile" ON public.profiles FOR UPDATE USING ((auth.uid() = id));


--
-- Name: robots Users can update their own robots; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Users can update their own robots" ON public.robots FOR UPDATE USING ((auth.uid() = user_id));


--
-- Name: alerts Users can view alerts for their own robots; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Users can view alerts for their own robots" ON public.alerts FOR SELECT USING ((EXISTS ( SELECT 1
   FROM public.robots
  WHERE ((robots.id = alerts.robot_id) AND (robots.user_id = auth.uid())))));


--
-- Name: telemetry Users can view telemetry for their own robots; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Users can view telemetry for their own robots" ON public.telemetry FOR SELECT USING ((EXISTS ( SELECT 1
   FROM public.robots
  WHERE ((robots.id = telemetry.robot_id) AND (robots.user_id = auth.uid())))));


--
-- Name: profiles Users can view their own profile; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Users can view their own profile" ON public.profiles FOR SELECT USING ((auth.uid() = id));


--
-- Name: robots Users can view their own robots; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Users can view their own robots" ON public.robots FOR SELECT USING ((auth.uid() = user_id));


--
-- Name: subscriptions Users can view their own subscriptions; Type: POLICY; Schema: public; Owner: postgres
--

CREATE POLICY "Users can view their own subscriptions" ON public.subscriptions FOR SELECT USING ((auth.uid() = user_id));


--
-- Name: admin_users; Type: ROW SECURITY; Schema: public; Owner: postgres
--

ALTER TABLE public.admin_users ENABLE ROW LEVEL SECURITY;

--
-- Name: alerts; Type: ROW SECURITY; Schema: public; Owner: postgres
--

ALTER TABLE public.alerts ENABLE ROW LEVEL SECURITY;

--
-- Name: plan_limits; Type: ROW SECURITY; Schema: public; Owner: postgres
--

ALTER TABLE public.plan_limits ENABLE ROW LEVEL SECURITY;

--
-- Name: profiles; Type: ROW SECURITY; Schema: public; Owner: postgres
--

ALTER TABLE public.profiles ENABLE ROW LEVEL SECURITY;

--
-- Name: robots; Type: ROW SECURITY; Schema: public; Owner: postgres
--

ALTER TABLE public.robots ENABLE ROW LEVEL SECURITY;

--
-- Name: subscriptions; Type: ROW SECURITY; Schema: public; Owner: postgres
--

ALTER TABLE public.subscriptions ENABLE ROW LEVEL SECURITY;

--
-- Name: telemetry; Type: ROW SECURITY; Schema: public; Owner: postgres
--

ALTER TABLE public.telemetry ENABLE ROW LEVEL SECURITY;

--
-- Name: SCHEMA public; Type: ACL; Schema: -; Owner: pg_database_owner
--

GRANT USAGE ON SCHEMA public TO postgres;
GRANT USAGE ON SCHEMA public TO anon;
GRANT USAGE ON SCHEMA public TO authenticated;
GRANT USAGE ON SCHEMA public TO service_role;


--
-- Name: FUNCTION auto_start_free_trial(); Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON FUNCTION public.auto_start_free_trial() TO anon;
GRANT ALL ON FUNCTION public.auto_start_free_trial() TO authenticated;
GRANT ALL ON FUNCTION public.auto_start_free_trial() TO service_role;


--
-- Name: FUNCTION check_if_admin(user_id uuid); Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON FUNCTION public.check_if_admin(user_id uuid) TO anon;
GRANT ALL ON FUNCTION public.check_if_admin(user_id uuid) TO authenticated;
GRANT ALL ON FUNCTION public.check_if_admin(user_id uuid) TO service_role;


--
-- Name: FUNCTION create_first_admin(admin_email text); Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON FUNCTION public.create_first_admin(admin_email text) TO anon;
GRANT ALL ON FUNCTION public.create_first_admin(admin_email text) TO authenticated;
GRANT ALL ON FUNCTION public.create_first_admin(admin_email text) TO service_role;


--
-- Name: FUNCTION get_alerts_by_robot(in_robot_id uuid); Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON FUNCTION public.get_alerts_by_robot(in_robot_id uuid) TO anon;
GRANT ALL ON FUNCTION public.get_alerts_by_robot(in_robot_id uuid) TO authenticated;
GRANT ALL ON FUNCTION public.get_alerts_by_robot(in_robot_id uuid) TO service_role;


--
-- Name: FUNCTION get_plan_limits(); Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON FUNCTION public.get_plan_limits() TO anon;
GRANT ALL ON FUNCTION public.get_plan_limits() TO authenticated;
GRANT ALL ON FUNCTION public.get_plan_limits() TO service_role;


--
-- Name: FUNCTION get_profile_by_id(profile_id uuid); Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON FUNCTION public.get_profile_by_id(profile_id uuid) TO anon;
GRANT ALL ON FUNCTION public.get_profile_by_id(profile_id uuid) TO authenticated;
GRANT ALL ON FUNCTION public.get_profile_by_id(profile_id uuid) TO service_role;


--
-- Name: FUNCTION get_robots_by_user(in_user_id uuid); Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON FUNCTION public.get_robots_by_user(in_user_id uuid) TO anon;
GRANT ALL ON FUNCTION public.get_robots_by_user(in_user_id uuid) TO authenticated;
GRANT ALL ON FUNCTION public.get_robots_by_user(in_user_id uuid) TO service_role;


--
-- Name: FUNCTION get_subscriptions_by_user(in_user_id uuid); Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON FUNCTION public.get_subscriptions_by_user(in_user_id uuid) TO anon;
GRANT ALL ON FUNCTION public.get_subscriptions_by_user(in_user_id uuid) TO authenticated;
GRANT ALL ON FUNCTION public.get_subscriptions_by_user(in_user_id uuid) TO service_role;


--
-- Name: FUNCTION get_telemetry_by_robot(in_robot_id uuid); Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON FUNCTION public.get_telemetry_by_robot(in_robot_id uuid) TO anon;
GRANT ALL ON FUNCTION public.get_telemetry_by_robot(in_robot_id uuid) TO authenticated;
GRANT ALL ON FUNCTION public.get_telemetry_by_robot(in_robot_id uuid) TO service_role;


--
-- Name: FUNCTION handle_new_user(); Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON FUNCTION public.handle_new_user() TO anon;
GRANT ALL ON FUNCTION public.handle_new_user() TO authenticated;
GRANT ALL ON FUNCTION public.handle_new_user() TO service_role;


--
-- Name: FUNCTION is_admin(in_uid uuid); Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON FUNCTION public.is_admin(in_uid uuid) TO anon;
GRANT ALL ON FUNCTION public.is_admin(in_uid uuid) TO authenticated;
GRANT ALL ON FUNCTION public.is_admin(in_uid uuid) TO service_role;


--
-- Name: FUNCTION update_robot_status(); Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON FUNCTION public.update_robot_status() TO anon;
GRANT ALL ON FUNCTION public.update_robot_status() TO authenticated;
GRANT ALL ON FUNCTION public.update_robot_status() TO service_role;


--
-- Name: FUNCTION update_updated_at_column(); Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON FUNCTION public.update_updated_at_column() TO anon;
GRANT ALL ON FUNCTION public.update_updated_at_column() TO authenticated;
GRANT ALL ON FUNCTION public.update_updated_at_column() TO service_role;


--
-- Name: TABLE admin_users; Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON TABLE public.admin_users TO anon;
GRANT ALL ON TABLE public.admin_users TO authenticated;
GRANT ALL ON TABLE public.admin_users TO service_role;


--
-- Name: TABLE alerts; Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON TABLE public.alerts TO anon;
GRANT ALL ON TABLE public.alerts TO authenticated;
GRANT ALL ON TABLE public.alerts TO service_role;


--
-- Name: TABLE plan_limits; Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON TABLE public.plan_limits TO anon;
GRANT ALL ON TABLE public.plan_limits TO authenticated;
GRANT ALL ON TABLE public.plan_limits TO service_role;


--
-- Name: TABLE profiles; Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON TABLE public.profiles TO anon;
GRANT ALL ON TABLE public.profiles TO authenticated;
GRANT ALL ON TABLE public.profiles TO service_role;


--
-- Name: TABLE robots; Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON TABLE public.robots TO anon;
GRANT ALL ON TABLE public.robots TO authenticated;
GRANT ALL ON TABLE public.robots TO service_role;


--
-- Name: TABLE subscriptions; Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON TABLE public.subscriptions TO anon;
GRANT ALL ON TABLE public.subscriptions TO authenticated;
GRANT ALL ON TABLE public.subscriptions TO service_role;


--
-- Name: TABLE telemetry; Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON TABLE public.telemetry TO anon;
GRANT ALL ON TABLE public.telemetry TO authenticated;
GRANT ALL ON TABLE public.telemetry TO service_role;


--
-- Name: TABLE user_admin_view; Type: ACL; Schema: public; Owner: postgres
--

GRANT ALL ON TABLE public.user_admin_view TO anon;
GRANT ALL ON TABLE public.user_admin_view TO authenticated;
GRANT ALL ON TABLE public.user_admin_view TO service_role;


--
-- Name: DEFAULT PRIVILEGES FOR SEQUENCES; Type: DEFAULT ACL; Schema: public; Owner: postgres
--

ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON SEQUENCES  TO postgres;
ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON SEQUENCES  TO anon;
ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON SEQUENCES  TO authenticated;
ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON SEQUENCES  TO service_role;


--
-- Name: DEFAULT PRIVILEGES FOR SEQUENCES; Type: DEFAULT ACL; Schema: public; Owner: supabase_admin
--

-- ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON SEQUENCES  TO postgres;
-- ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON SEQUENCES  TO anon;
-- ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON SEQUENCES  TO authenticated;
-- ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON SEQUENCES  TO service_role;


--
-- Name: DEFAULT PRIVILEGES FOR FUNCTIONS; Type: DEFAULT ACL; Schema: public; Owner: postgres
--

ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON FUNCTIONS  TO postgres;
ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON FUNCTIONS  TO anon;
ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON FUNCTIONS  TO authenticated;
ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON FUNCTIONS  TO service_role;


--
-- Name: DEFAULT PRIVILEGES FOR FUNCTIONS; Type: DEFAULT ACL; Schema: public; Owner: supabase_admin
--

-- ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON FUNCTIONS  TO postgres;
-- ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON FUNCTIONS  TO anon;
-- ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON FUNCTIONS  TO authenticated;
-- ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON FUNCTIONS  TO service_role;


--
-- Name: DEFAULT PRIVILEGES FOR TABLES; Type: DEFAULT ACL; Schema: public; Owner: postgres
--

ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON TABLES  TO postgres;
ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON TABLES  TO anon;
ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON TABLES  TO authenticated;
ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON TABLES  TO service_role;


--
-- Name: DEFAULT PRIVILEGES FOR TABLES; Type: DEFAULT ACL; Schema: public; Owner: supabase_admin
--

-- ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON TABLES  TO postgres;
-- ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON TABLES  TO anon;
-- ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON TABLES  TO authenticated;
-- ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON TABLES  TO service_role;


--
-- PostgreSQL database dump complete
--

