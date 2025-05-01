
const fetch = require('node-fetch');

// Configuration - Update these values with your own
const ROBOT_ID = '00f48d5f-d82f-471b-afc8-05faaa1075ab'; // Replace with your robot ID
const API_KEY = ''; // Replace with your API key from ApiKeySettings in the dashboard
const SUPABASE_URL = 'https://uwmbdporlrduzthgdmcg.supabase.co';

// Send telemetry data every 10 seconds by default
const INTERVAL_MS = 10000;

// Initial values
let batteryLevel = 80;
let temperature = 25;
let latitude = 33.748;
let longitude = -84.387;

async function sendTelemetry() {
  // Generate telemetry data
  batteryLevel = Math.max(0, Math.min(100, batteryLevel - (Math.random() * 2)));
  temperature = Math.max(20, Math.min(50, temperature + ((Math.random() * 2) - 1)));
  
  // Move the robot slightly
  latitude += (Math.random() - 0.5) * 0.005;
  longitude += (Math.random() - 0.5) * 0.005;
  
  // Determine status - occasionally generate warnings or errors
  const rand = Math.random();
  let status = "OK";
  let errorCodes = [];
  let warningCodes = [];
  
  if (rand > 0.95) {
    status = "ERROR";
    errorCodes = [`ERR_${Math.floor(Math.random() * 1000).toString().padStart(3, '0')}`];
  } else if (rand > 0.85 || batteryLevel < 20 || temperature > 40) {
    status = "WARNING";
    warningCodes = [`WARN_${Math.floor(Math.random() * 1000).toString().padStart(3, '0')}`];
  }

  // Create telemetry payload
  const telemetry = {
    robotId: ROBOT_ID,
    batteryLevel,
    temperature,
    status,
    errorCodes,
    warningCodes,
    location: {
      latitude,
      longitude
    },
    timestamp: new Date().toISOString()
  };

  console.log('Generating telemetry:', JSON.stringify(telemetry, null, 2));

  try {
    // Send to telemetry endpoint
    const headers = {
      'Content-Type': 'application/json',
      'api-key': API_KEY
    };
    
    console.log('Sending with headers:', {
      'Content-Type': 'application/json',
      'api-key': API_KEY ? 'PRESENT (hidden for security)' : 'MISSING!'
    });
    
    const endpoint = `${SUPABASE_URL}/functions/v1/telemetry`;
    console.log('Sending to:', endpoint);
    
    const response = await fetch(endpoint, {
      method: 'POST',
      headers,
      body: JSON.stringify(telemetry)
    });

    console.log('Response status:', response.status, response.statusText);
    console.log('Response headers:', response.headers.raw());
    
    const responseData = await response.json();
    
    if (!response.ok) {
      console.log('❌ Failed to send telemetry:', response.status, response.statusText);
      console.log('Error details:', responseData);
    } else {
      console.log('✅ Telemetry sent successfully!');
      console.log('Response:', responseData);
    }
  } catch (error) {
    console.error('❌ Error sending telemetry:', error);
  }
}

console.log('🤖 Robot Telemetry Simulator');
console.log('============================');
console.log(`Robot ID: ${ROBOT_ID}`);
console.log(`API Key: ${API_KEY ? '********' + API_KEY.substring(API_KEY.length - 4) : 'NOT SET! Update the API_KEY value in this script'}`);
console.log(`Endpoint: ${SUPABASE_URL}/functions/v1/telemetry`);
console.log(`Interval: ${INTERVAL_MS / 1000} seconds`);
console.log('============================\n');

// First telemetry send
sendTelemetry();

// Then schedule regular sends
setInterval(sendTelemetry, INTERVAL_MS);
