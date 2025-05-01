
// robot-simulator.js
// Run with: node robot-simulator.js

const fetch = (...args) => import('node-fetch').then(({default: fetch}) => fetch(...args));

// Configuration
const API_URL = "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry";
const ROBOT_ID = "YOUR_ROBOT_ID"; // Replace with your actual robot ID
const API_KEY = "YOUR_ROBOT_API_KEY"; // Replace with your actual robot's API key
const SIMULATION_INTERVAL = 5000; // 5 seconds between data sends

// Function to generate random telemetry data
function generateTelemetry() {
  // Random battery between 20% and 100%
  const batteryLevel = Math.floor(Math.random() * 81) + 20;
  
  // Random temperature between 20°C and 35°C
  const temperature = parseFloat((Math.random() * 15 + 20).toFixed(1));
  
  // Random status (mostly OK, occasionally WARNING or ERROR)
  const statusRoll = Math.random();
  let status = "OK";
  if (statusRoll > 0.9) status = "WARNING";
  if (statusRoll > 0.97) status = "ERROR";
  
  // Random error codes for ERROR status
  const errorCodes = status === "ERROR" ? ["E001", "E003"] : [];
  
  // Random warning codes for WARNING status
  const warningCodes = status === "WARNING" ? ["W002"] : [];
  
  // Random location near Atlanta (with small variations)
  const location = {
    latitude: 33.748995 + (Math.random() - 0.5) * 0.01, 
    longitude: -84.387982 + (Math.random() - 0.5) * 0.01
  };
  
  return {
    robotId: ROBOT_ID,
    batteryLevel,
    temperature,
    status,
    errorCodes,
    warningCodes,
    location,
    timestamp: new Date().toISOString()
  };
}

// Function to send telemetry data
async function sendTelemetry() {
  const data = generateTelemetry();
  console.log(`Generating telemetry: ${JSON.stringify(data, null, 2)}`);
  
  try {
    // Show all headers being sent for debugging
    const headers = {
      "Content-Type": "application/json",
      "api-key": API_KEY
    };
    
    console.log("Sending headers:", headers);
    
    const response = await fetch(API_URL, {
      method: "POST",
      headers,
      body: JSON.stringify(data)
    });
    
    // Log the raw response
    console.log(`Response status: ${response.status} ${response.statusText}`);
    
    const result = await response.json();
    
    if (response.ok) {
      console.log(`✅ Telemetry sent successfully: ${JSON.stringify(result)}`);
    } else {
      console.error(`❌ Failed to send telemetry: ${response.status} ${response.statusText}`);
      console.error(result);
    }
  } catch (error) {
    console.error(`❌ Error sending telemetry: ${error.message}`);
  }
}

// Start simulation
console.log(`🤖 Robot Simulator starting for robot ${ROBOT_ID}`);
console.log(`📡 Sending telemetry every ${SIMULATION_INTERVAL/1000} seconds...`);
console.log(`📍 API URL: ${API_URL}`);
console.log(`🔑 Using API key: ${API_KEY ? "Set (not shown for security)" : "NOT SET - WILL FAIL"}`);

// Show warning if the robot ID or API key haven't been set
if (ROBOT_ID === "YOUR_ROBOT_ID" || API_KEY === "YOUR_ROBOT_API_KEY") {
  console.error(`
⚠️  WARNING: You need to replace the placeholder values:
   - Replace YOUR_ROBOT_ID with your actual robot ID 
   - Replace YOUR_ROBOT_API_KEY with your actual robot's API key
   Both can be found in your RoboMonitor dashboard under "API Integration"
  `);
}

// Initial send
sendTelemetry();

// Set up interval for regular sends
setInterval(sendTelemetry, SIMULATION_INTERVAL);
