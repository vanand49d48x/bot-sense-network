
# Robot Telemetry Simulator Client

This is a simple Node.js client that simulates a robot sending telemetry data to your Robometrics platform.

## Setup

1. Make sure you have Node.js installed (version 14 or higher recommended)

2. Install dependencies:
   ```
   npm install node-fetch@2
   ```

3. Edit the `robot-client.js` file and update these values:
   - `ROBOT_ID`: The ID of your robot (found in the dashboard)
   - `API_KEY`: Your API key (found in the API Key Settings panel in the sidebar)

## Running the client

Run the client with:

```
node robot-client.js
```

The client will:
- Send an initial telemetry message immediately
- Continue sending telemetry every 10 seconds (configurable in the script)
- Simulate battery drain, temperature changes, and location movement
- Occasionally generate warnings or errors

## Troubleshooting

### API Key Guidelines
- The API Key must be provided in the `api-key` header
- The API Key must match the one shown in your API Key Settings panel in the sidebar
- This is a user-level API key, not a robot-specific key

### If you see a 401 Unauthorized error, check:
1. That you've entered the correct API key from the API Key panel in the sidebar
2. That the robot ID you're using belongs to your account
3. That the headers are properly formatted in the request
4. Check the Supabase logs for the telemetry function for more information

## How to get your API key

1. Login to your Robometrics dashboard
2. Look for the API Key panel in the sidebar
3. Copy the key shown there to use in this client

## How to get your robot ID

1. In your dashboard, click on one of your robots
2. Look for "Robot ID" in the details section, or check the URL which includes the robot ID
