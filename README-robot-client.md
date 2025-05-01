
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

If you see a 401 Unauthorized error, check:
1. That you've entered the correct API key from your profile
2. That the robot ID you're using belongs to your account

## How to get your API key

1. Login to your Robometrics dashboard
2. Look for the API Key panel in the sidebar
3. Copy the key shown there to use in this client

## How to get your robot ID

1. In your dashboard, click on one of your robots
2. Look for "Robot ID" in the details section, or check the URL which includes the robot ID
