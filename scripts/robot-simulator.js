
// This is a simple Node.js script to simulate robots sending telemetry data
import fetch from "node-fetch";

// Configuration
const API_URL = "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry";
const API_KEY = "0adc8b5293cd4c11b98058a3ebbb474cd8b17dae5ddc4ee599af9664020ff664";

// List of robot IDs - replace with your actual robot IDs from the database
const robotIds = [
  "1b9b4561-e37a-481a-aafd-9a850dfea05f",
  "f3c88253-cfbb-42e4-b61b-63663e18d4d8",
  "2ca2a29c-b28d-4926-b3e2-5f06134b1ea5",
  "ab0cefa0-0e90-4576-b6c3-214e37043107",
  "00f48d5f-d82f-471b-afc8-05faaa1075ab"
];

// Function to generate random telemetry data
function generateRandomTelemetry(robotId) {
  return {
    robotId: robotId,
    batteryLevel: Math.floor(Math.random() * 100),
    temperature: parseFloat((20 + Math.random() * 40).toFixed(1)),
    status: randomStatus(),
    location: {
      // Important: Use latitude/longitude format instead of lat/lng
      latitude: 33.7756 + (Math.random() - 0.5) * 0.01, 
      longitude: -84.3963 + (Math.random() - 0.5) * 0.01
    },
    timestamp: new Date().toISOString()
  };
}

// Function to generate random status with weighted distribution
function randomStatus() {
  const statuses = ["OK", "WARNING", "ERROR"];
  const weights = [0.8, 0.15, 0.05]; // 80% OK, 15% WARNING, 5% ERROR
  
  const random = Math.random();
  let weightSum = 0;
  
  for (let i = 0; i < weights.length; i++) {
    weightSum += weights[i];
    if (random <= weightSum) return statuses[i];
  }
  
  return statuses[0];
}

// Function to send telemetry for a specific robot
async function sendTelemetry(robotId) {
  const data = generateRandomTelemetry(robotId);

  try {
    const res = await fetch(API_URL, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        "api-key": API_KEY // Important: Use api-key instead of Authorization
      },
      body: JSON.stringify(data)
    });

    const response = await res.json();
    console.log(`📡 Sent telemetry for ${robotId}:`, data, "\nResponse:", response);
    return response;
  } catch (error) {
    console.error(`❌ Error sending telemetry for ${robotId}:`, error);
    return { error: error.message };
  }
}

// Function to simulate all robots sending data once
async function simulateAllRobots() {
  console.log(`Starting telemetry simulation for ${robotIds.length} robots...`);
  
  for (const robotId of robotIds) {
    await sendTelemetry(robotId);
    // Add a small delay between requests to avoid overwhelming the server
    await new Promise(resolve => setTimeout(resolve, 500));
  }
  
  console.log("Simulation complete!");
}

// Function to start continuous simulation
function startContinuousSimulation(intervalSeconds = 10) {
  console.log(`Starting continuous simulation with ${intervalSeconds} second intervals`);
  
  // Send initial batch
  simulateAllRobots();
  
  // Set up intervals for each robot with a slight offset
  robotIds.forEach((robotId, index) => {
    // Add a small offset for each robot to distribute the load
    const offsetMs = index * 1000;
    
    setTimeout(() => {
      setInterval(() => {
        sendTelemetry(robotId);
      }, intervalSeconds * 1000);
    }, offsetMs);
  });
  
  console.log(`Simulation running! Sending data every ${intervalSeconds} seconds for each robot.`);
  console.log("Press Ctrl+C to stop the simulation.");
}

// Run a single simulation by default
// To run continuously, change this to startContinuousSimulation(10) 
// where 10 is the interval in seconds
simulateAllRobots();

// Uncomment the line below to start continuous simulation instead
// startContinuousSimulation(10);
