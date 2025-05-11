
# Environment Configuration for Supabase

This project is set up to work with two different Supabase environments:

## Environments

1. **Development Environment (DEV)**
   - Project Reference: rtcspemkxqiecoqeeuai
   - URL: https://rtcspemkxqiecoqeeuai.supabase.co
   - Branch: `main`

2. **Production Environment (PROD)**
   - Project Reference: uwmbdporlrduzthgdmcg
   - URL: https://uwmbdporlrduzthgdmcg.supabase.co
   - Branch: `production`

## How It Works

### Environment Selection

The application automatically selects the appropriate Supabase environment based on:

- In development mode (local development): Uses the development environment
- In production mode (built with `NODE_ENV=production`): Uses the production environment
- When deployed via GitHub Actions: Uses environment based on the branch (main -> dev, production -> prod)

### Edge Functions

Edge functions use environment variables set during deployment to determine which Supabase project to connect to.

### GitHub Actions Workflow

The GitHub Actions workflow in `.github/workflows/deploy.yml` automatically:
1. Detects the branch (main or production)
2. Sets the appropriate environment variables
3. Deploys functions to the corresponding Supabase project
4. Builds the application with the correct environment settings

### Local Development

For local development:
- The application defaults to the development environment
- You can view which environment is active using the environment indicator badge (visible only in development mode)

## Ensuring Consistent Schemas

To ensure both environments have consistent database schemas:

1. Always apply migrations to both environments
2. Use the Supabase CLI to manage migrations
3. Test in development before deploying to production

## Manual Environment Override

For testing production environment locally, modify the `ACTIVE_ENVIRONMENT` value in `src/integrations/supabase/client.ts`.
