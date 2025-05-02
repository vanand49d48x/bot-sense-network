
const fetch = require('node-fetch');

// Configuration - Update these values with your own
const ROBOT_ID = '00f48d5f-d82f-471b-afc8-05faaa1075ab'; // Replace with your robot ID
const API_KEY = 'ca5c45b3fd7144ce87f93a7ba3469735a54006da4210450da6e4d24ca5b14e1e'; // Replace with your API key from the API Key panel in the sidebar
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

  console.log('========================================');
  console.log(`Generating telemetry for robot: ${ROBOT_ID}`);
  console.log('Data:', JSON.stringify(telemetry, null, 2));

  try {
    if (!API_KEY) {
      console.error('❌ API_KEY is not set! Please update the API_KEY value in this script.');
      console.error('   You can find your API key in the API Key panel in the sidebar.');
      return;
    }

    // Send to telemetry endpoint
    const headers = {
      'Content-Type': 'application/json',
      'api-key': API_KEY
    };
    
    console.log('Sending with headers:', {
      'Content-Type': 'application/json',
      'api-key': API_KEY ? `${API_KEY.substring(0, 5)}...${API_KEY.substring(API_KEY.length - 5)}` : 'MISSING!'
    });
    
    const endpoint = `${SUPABASE_URL}/functions/v1/telemetry`;
    console.log('Sending to:', endpoint);
    console.log('----------------------------------------');
    
    const response = await fetch(endpoint, {
      method: 'POST',
      headers,
      body: JSON.stringify(telemetry)
    });

    console.log('Response status:', response.status, response.statusText);
    
    // Log all response headers for debugging
    console.log('Response headers:', Object.fromEntries([...Object.entries(response.headers.raw())]));
    
    let responseData;
    try {
      responseData = await response.json();
      console.log('Response data:', responseData);
    } catch (error) {
      console.log('Could not parse response as JSON:', error.message);
      const text = await response.text();
      console.log('Raw response:', text);
    }
    
    if (!response.ok) {
      console.log('❌ Failed to send telemetry:', response.status, response.statusText);
      if (responseData) console.log('Error details:', responseData);
      console.log('\nTroubleshooting tips:');
      console.log('1. Check that your API_KEY is correctly set in this file');
      console.log('2. Verify the robot ID belongs to your account');
      console.log('3. Check the telemetry function logs in Supabase');
    } else {
      console.log('✅ Telemetry sent successfully!');
      if (responseData) console.log('Response:', responseData);
    }
  } catch (error) {
    console.error('❌ Error sending telemetry:', error);
    console.log('\nTroubleshooting tips:');
    console.log('1. Check your internet connection');
    console.log('2. Verify the SUPABASE_URL is correct');
    console.log('3. Make sure you have the right permissions');
    console.log('4. Inspect network requests in browser devtools for more info');
  }
  console.log('========================================\n');
}

console.log('🤖 Robot Telemetry Simulator');
console.log('============================');
console.log(`Robot ID: ${ROBOT_ID}`);
console.log(`API Key: ${API_KEY ? '********' + API_KEY.substring(API_KEY.length - 4) : 'NOT SET! Please update the API_KEY value in this script'}`);
console.log(`Endpoint: ${SUPABASE_URL}/functions/v1/telemetry`);
console.log(`Interval: ${INTERVAL_MS / 1000} seconds`);
console.log('============================\n');

// First telemetry send
sendTelemetry();

// Then schedule regular sends
setInterval(sendTelemetry, INTERVAL_MS);
