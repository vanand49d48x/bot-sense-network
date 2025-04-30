
# RobometricsMessage.msg
# Message format for RoboMonitor telemetry data

# Robot identifier
string robot_id

# Battery level (0-100)
int32 battery_level

# Temperature in Celsius
float32 temperature

# Status string ("OK", "WARNING", "ERROR")
string status

# Location coordinates
float64 latitude
float64 longitude

# Optional timestamp (if not provided, current time will be used)
time timestamp
