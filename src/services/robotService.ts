
import { Robot } from "../types/robot";

const generateRandomData = (): Robot[] => {
  const robots: Robot[] = [
    {
      id: "r-001",
      name: "Warehouse Bot 1",
      model: "PickAssist X1",
      robotType: "PickAssist X1",
      status: "online",
      lastHeartbeat: new Date().toISOString(),
      batteryLevel: 87,
      temperature: 32,
      location: {
        latitude: 37.7749,
        longitude: -122.4194,
      },
      ipAddress: "192.168.1.101",
      errorCount: 0,
      created_at: new Date().toISOString(),
      description: "Warehouse assistant robot",
      apiKey: "robot-key-001", // Added required apiKey
    },
    {
      id: "r-002",
      name: "Delivery Drone 5",
      model: "SkyRunner D5",
      robotType: "SkyRunner D5",
      status: "online",
      lastHeartbeat: new Date().toISOString(),
      batteryLevel: 65,
      temperature: 28,
      location: {
        latitude: 37.7833,
        longitude: -122.4167,
      },
      ipAddress: "192.168.1.102",
      errorCount: 0,
      created_at: new Date().toISOString(),
      description: "Aerial delivery drone",
      apiKey: "robot-key-002", // Added required apiKey
    },
    {
      id: "r-003",
      name: "Cleaning Bot 3",
      model: "FloorMaster C3",
      robotType: "FloorMaster C3",
      status: "warning",
      lastHeartbeat: new Date().toISOString(),
      batteryLevel: 22,
      temperature: 37,
      location: {
        latitude: 37.7695,
        longitude: -122.4143,
      },
      ipAddress: "192.168.1.103",
      errorCount: 2,
      created_at: new Date().toISOString(),
      description: "Automated cleaning robot",
      apiKey: "robot-key-003", // Added required apiKey
    },
    {
      id: "r-004",
      name: "Security Bot 7",
      model: "GuardianPro S7",
      robotType: "GuardianPro S7",
      status: "offline",
      lastHeartbeat: new Date(Date.now() - 15 * 60000).toISOString(), // 15 min ago
      batteryLevel: 0,
      temperature: 0,
      location: {
        latitude: 37.7700,
        longitude: -122.4120,
      },
      ipAddress: "192.168.1.104",
      errorCount: 5,
      created_at: new Date().toISOString(),
      description: "Security patrol robot",
      apiKey: "robot-key-004", // Added required apiKey
    },
    {
      id: "r-005",
      name: "Assembly Bot 2",
      model: "BuilderX A2",
      robotType: "BuilderX A2",
      status: "online",
      lastHeartbeat: new Date().toISOString(),
      batteryLevel: 92,
      temperature: 30,
      ipAddress: "192.168.1.105",
      errorCount: 0,
      location: {
        latitude: 37.7800,
        longitude: -122.4150,
      },
      created_at: new Date().toISOString(),
      description: "Manufacturing assembly robot",
      apiKey: "robot-key-005", // Added required apiKey
    },
  ];
  
  return robots;
};

let robots = generateRandomData();

const updateRobotData = () => {
  robots = robots.map((robot) => {
    if (robot.status === 'offline') return robot;
    
    // Update last heartbeat for online bots
    const newRobot = { 
      ...robot,
      lastHeartbeat: new Date().toISOString() 
    };
    
    // Randomly update battery level (slight decrease)
    if (Math.random() > 0.7) {
      newRobot.batteryLevel = Math.max(
        0, 
        robot.batteryLevel - Math.random() * 2
      );
    }
    
    // Randomly update temperature
    if (Math.random() > 0.6) {
      const tempChange = (Math.random() * 2 - 1);
      newRobot.temperature = Math.round((robot.temperature + tempChange) * 10) / 10;
    }
    
    // Update status based on battery
    if (newRobot.batteryLevel < 20 && newRobot.status !== 'warning') {
      newRobot.status = 'warning';
    }
    
    // Randomly update location for robots that have location
    if (robot.location && Math.random() > 0.5) {
      const latChange = (Math.random() - 0.5) * 0.001;
      const lngChange = (Math.random() - 0.5) * 0.001;
      newRobot.location = {
        latitude: robot.location.latitude + latChange,
        longitude: robot.location.longitude + lngChange
      };
    }
    
    return newRobot;
  });
  
  return robots;
};

const getRobots = (): Robot[] => {
  return updateRobotData();
};

const getRobotById = (id: string): Robot | undefined => {
  return robots.find((robot) => robot.id === id);
};

export const robotService = {
  getRobots,
  getRobotById,
};
