
# Environment Configuration for Supabase

This project is set up to work with environment variables for connecting to Supabase:

## Required Environment Variables

The following environment variables **MUST** be set for the application to work:

- `VITE_SUPABASE_URL`: The URL of your Supabase project
- `VITE_SUPABASE_ANON_KEY`: The anon key of your Supabase project

## Environment Selection

The application automatically selects the appropriate Supabase environment based on:

- In development mode (local development): Uses environment variables provided in `.env.local` or during development server startup
- In production mode (built with `NODE_ENV=production`): Uses environment variables set during build time
- When deployed via GitHub Actions: Uses environment variables set from GitHub secrets

## GitHub Actions Configuration

For deployment to work correctly, the following secrets must be set in your GitHub repository:

- `DEV_SUPABASE_URL`: URL for the development Supabase instance
- `DEV_SUPABASE_ANON_KEY`: Anon key for the development Supabase instance
- `PROD_SUPABASE_URL`: URL for the production Supabase instance
- `PROD_SUPABASE_ANON_KEY`: Anon key for the production Supabase instance
- `SUPABASE_ACCESS_TOKEN`: Access token for deploying edge functions

## Edge Functions

Edge functions use environment variables set during deployment to determine which Supabase project to connect to.

## GitHub Actions Workflow

The GitHub Actions workflow in `.github/workflows/deploy.yml` automatically:
1. Detects the branch (main or production)
2. Sets the appropriate environment variables from GitHub secrets
3. Deploys functions to the corresponding Supabase project
4. Builds the application with the correct environment settings

## Local Development

For local development:
1. Create a `.env.local` file with your Supabase URL and anon key:
   ```
   VITE_SUPABASE_URL=your_supabase_url
   VITE_SUPABASE_ANON_KEY=your_supabase_anon_key
   ```
2. The application will use these variables to connect to your Supabase project
3. You can view which environment is active using the environment indicator badge (visible only in development mode)

## Ensuring Consistent Schemas

To ensure both environments have consistent database schemas:

1. Always apply migrations to both environments
2. Use the Supabase CLI to manage migrations
3. Test in development before deploying to production
