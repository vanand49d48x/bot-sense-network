import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { useState } from "react";
import { toast } from "@/components/ui/sonner";
import { ClipboardCopy, Download } from "lucide-react";

export function ArduinoExample() {
  const [activeTab, setActiveTab] = useState("arduino");

  const arduinoExample = `
#include <Arduino.h>
#include <WiFiNINA.h>  // For Arduino boards with WiFi built-in
#include <ArduinoJson.h>
#include <HTTPClient.h>

// RoboMetrics configuration
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* API_ENDPOINT = "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry";
const char* ROBOT_ID = "YOUR_ROBOT_ID";
const char* API_KEY = "YOUR_ROBOT_API_KEY";

// Sensor pins
const int BATTERY_PIN = A0;
const int TEMPERATURE_PIN = A1;

// Variables for sensors
float batteryVoltage = 0.0;
float batteryPercentage = 0.0;
float temperature = 0.0;

// Position (update these with GPS or other positioning if available)
float latitude = 37.7749;
float longitude = -122.4194;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Connect to WiFi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Read sensors
  readSensors();
  
  // Send telemetry data
  sendTelemetry();
  
  // Wait 60 seconds before next update
  delay(60000);
}

void readSensors() {
  // Read battery voltage from analog pin
  int batteryRaw = analogRead(BATTERY_PIN);
  // Convert to voltage (example assumes 3.3V reference and voltage divider)
  batteryVoltage = batteryRaw * (3.3 / 1023.0) * 2; // Multiply by 2 if using voltage divider
  // Convert to percentage (example assumes 3.0V is 0% and 4.2V is 100% for LiPo)
  batteryPercentage = map(batteryVoltage * 100, 300, 420, 0, 100);
  batteryPercentage = constrain(batteryPercentage, 0, 100);
  
  // Read temperature from analog pin
  int tempRaw = analogRead(TEMPERATURE_PIN);
  // Example using TMP36 sensor
  float voltage = tempRaw * (3.3 / 1023.0);
  temperature = (voltage - 0.5) * 100; // TMP36 formula
  
  Serial.print("Battery: ");
  Serial.print(batteryPercentage);
  Serial.println("%");
  
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("°C");
}

void sendTelemetry() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    return;
  }
  
  HTTPClient http;
  http.begin(API_ENDPOINT);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("api-key", API_KEY);
  
  // Create JSON document
  StaticJsonDocument<300> doc;
  doc["robotId"] = ROBOT_ID;
  doc["batteryLevel"] = batteryPercentage;
  doc["temperature"] = temperature;
  doc["status"] = "OK";
  
  JsonObject location = doc.createNestedObject("location");
  location["latitude"] = latitude;
  location["longitude"] = longitude;
  
  // Serialize JSON to string
  String requestBody;
  serializeJson(doc, requestBody);
  
  Serial.println("Sending telemetry data...");
  Serial.println(requestBody);
  
  // Send POST request
  int httpResponseCode = http.POST(requestBody);
  
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("HTTP Response code: " + String(httpResponseCode));
    Serial.println(response);
  } else {
    Serial.print("Error on sending request: ");
    Serial.println(httpResponseCode);
  }
  
  http.end();
}`;

  const esp32Example = `
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// RoboMetrics configuration
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* API_ENDPOINT = "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry";
const char* ROBOT_ID = "YOUR_ROBOT_ID";
const char* API_KEY = "YOUR_ROBOT_API_KEY";

// Sensor pins
const int BATTERY_PIN = 34; // ESP32 ADC pin
const int TEMPERATURE_PIN = 35; // ESP32 ADC pin

// Variables for sensors
float batteryVoltage = 0.0;
float batteryPercentage = 0.0;
float temperature = 0.0;

// Position (update these with GPS or other positioning if available)
float latitude = 37.7749;
float longitude = -122.4194;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Connect to WiFi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Read sensors
  readSensors();
  
  // Send telemetry data
  sendTelemetry();
  
  // Wait 60 seconds before next update
  delay(60000);
}

void readSensors() {
  // Read battery voltage from analog pin
  int batteryRaw = analogRead(BATTERY_PIN);
  // Convert to voltage (ESP32 ADC is 12-bit, 0-4095)
  batteryVoltage = batteryRaw * (3.3 / 4095.0) * 2; // Multiply by 2 if using voltage divider
  // Convert to percentage (example assumes 3.0V is 0% and 4.2V is 100% for LiPo)
  batteryPercentage = map(batteryVoltage * 100, 300, 420, 0, 100);
  batteryPercentage = constrain(batteryPercentage, 0, 100);
  
  // Read temperature
  int tempRaw = analogRead(TEMPERATURE_PIN);
  // Example using TMP36 sensor
  float voltage = tempRaw * (3.3 / 4095.0);
  temperature = (voltage - 0.5) * 100; // TMP36 formula
  
  Serial.print("Battery: ");
  Serial.print(batteryPercentage);
  Serial.println("%");
  
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("°C");
}

void sendTelemetry() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    return;
  }
  
  HTTPClient http;
  http.begin(API_ENDPOINT);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("api-key", API_KEY);
  
  // Create JSON document
  StaticJsonDocument<300> doc;
  doc["robotId"] = ROBOT_ID;
  doc["batteryLevel"] = batteryPercentage;
  doc["temperature"] = temperature;
  doc["status"] = "OK";
  
  JsonObject location = doc.createNestedObject("location");
  location["latitude"] = latitude;
  location["longitude"] = longitude;
  
  // Serialize JSON to string
  String requestBody;
  serializeJson(doc, requestBody);
  
  Serial.println("Sending telemetry data...");
  Serial.println(requestBody);
  
  // Send POST request
  int httpResponseCode = http.POST(requestBody);
  
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("HTTP Response code: " + String(httpResponseCode));
    Serial.println(response);
  } else {
    Serial.print("Error on sending request: ");
    Serial.println(httpResponseCode);
  }
  
  http.end();
}`;

  const libraryCode = `
#include "RoboMetrics.h"

RoboMetrics::RoboMetrics(const char* robotId, const char* apiKey) {
  _robotId = robotId;
  _apiKey = apiKey;
  _apiEndpoint = "https://uwmbdporlrduzthgdmcg.supabase.co/functions/v1/telemetry";
  _batteryLevel = 100;
  _temperature = 25;
  _status = "OK";
  _hasLocation = false;
}

void RoboMetrics::setWiFi(const char* ssid, const char* password) {
  _ssid = ssid;
  _password = password;
}

void RoboMetrics::connect() {
  #if defined(ESP8266) || defined(ESP32)
    WiFi.begin(_ssid, _password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      attempts++;
    }
  #elif defined(ARDUINO_ARCH_SAMD) // For Arduino MKR WiFi 1010, etc.
    WiFi.begin(_ssid, _password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      attempts++;
    }
  #endif
}

void RoboMetrics::setBatteryLevel(float level) {
  _batteryLevel = level;
}

void RoboMetrics::setTemperature(float temp) {
  _temperature = temp;
}

void RoboMetrics::setLocation(float latitude, float longitude) {
  _latitude = latitude;
  _longitude = longitude;
  _hasLocation = true;
}

void RoboMetrics::setStatus(const char* status) {
  _status = status;
}

bool RoboMetrics::sendTelemetry() {
  #if defined(ESP8266) || defined(ESP32) || defined(ARDUINO_ARCH_SAMD)
    if (WiFi.status() != WL_CONNECTED) {
      connect();
      if (WiFi.status() != WL_CONNECTED) {
        return false;
      }
    }
    
    HTTPClient http;
    http.begin(_apiEndpoint);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("api-key", _apiKey);
    
    StaticJsonDocument<300> doc;
    doc["robotId"] = _robotId;
    doc["batteryLevel"] = _batteryLevel;
    doc["temperature"] = _temperature;
    doc["status"] = _status;
    
    if (_hasLocation) {
      JsonObject location = doc.createNestedObject("location");
      location["latitude"] = _latitude;
      location["longitude"] = _longitude;
    }
    
    String requestBody;
    serializeJson(doc, requestBody);
    
    int httpResponseCode = http.POST(requestBody);
    bool success = httpResponseCode > 0;
    http.end();
    
    return success;
  #else
    // Other Arduino boards not supported yet
    return false;
  #endif
}`;

  const libraryHeader = `
#ifndef RoboMetrics_h
#define RoboMetrics_h

#include <Arduino.h>

#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESP8266HTTPClient.h>
#elif defined(ESP32)
  #include <WiFi.h>
  #include <HTTPClient.h>
#elif defined(ARDUINO_ARCH_SAMD)
  #include <WiFiNINA.h>
  #include <ArduinoHttpClient.h>
#endif

#include <ArduinoJson.h>

class RoboMetrics {
  public:
    RoboMetrics(const char* robotId, const char* apiKey);
    void setWiFi(const char* ssid, const char* password);
    void connect();
    void setBatteryLevel(float level);
    void setTemperature(float temp);
    void setLocation(float latitude, float longitude);
    void setStatus(const char* status);
    bool sendTelemetry();
    
  private:
    const char* _robotId;
    const char* _apiKey;
    const char* _apiEndpoint;
    const char* _ssid;
    const char* _password;
    float _batteryLevel;
    float _temperature;
    const char* _status;
    float _latitude;
    float _longitude;
    bool _hasLocation;
};

#endif`;

  const simpleExample = `
#include "RoboMetrics.h"

// RoboMetrics configuration
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* ROBOT_ID = "YOUR_ROBOT_ID";
const char* API_KEY = "YOUR_ROBOT_API_KEY";

// Create RoboMetrics instance
RoboMetrics robot(ROBOT_ID, API_KEY);

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Connect to WiFi
  Serial.println("Connecting to WiFi...");
  robot.setWiFi(WIFI_SSID, WIFI_PASSWORD);
  robot.connect();
}

void loop() {
  // Update sensor values
  float batteryLevel = readBatteryLevel(); // Your function to read battery
  float temperature = readTemperature(); // Your function to read temperature
  
  // Set values in RoboMetrics
  robot.setBatteryLevel(batteryLevel);
  robot.setTemperature(temperature);
  robot.setLocation(37.7749, -122.4194); // Example location
  
  // Send telemetry data
  bool success = robot.sendTelemetry();
  if (success) {
    Serial.println("Telemetry sent successfully");
  } else {
    Serial.println("Failed to send telemetry");
  }
  
  // Wait 60 seconds before next update
  delay(60000);
}

float readBatteryLevel() {
  // Example function to read battery level (0-100)
  return 75.0;
}

float readTemperature() {
  // Example function to read temperature in Celsius
  return 25.5;
}
`;

  const copyToClipboard = (text: string, type: string) => {
    navigator.clipboard.writeText(text);
    toast.success(`${type} code copied to clipboard`);
  };

  const downloadFile = (content: string, fileName: string, fileType: string) => {
    const blob = new Blob([content], { type: fileType });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = fileName;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    toast.success(`${fileName} downloaded`);
  };

  return (
    <Card>
      <CardHeader>
        <CardTitle>Arduino/ESP32 Integration</CardTitle>
        <CardDescription>
          Use these code examples to connect Arduino or ESP32 devices to RoboMetrics. The examples show how to read sensor data and send telemetry to the platform.
        </CardDescription>
      </CardHeader>
      <CardContent>
        <Tabs defaultValue="arduino" value={activeTab} onValueChange={setActiveTab}>
          <TabsList className="mb-4">
            <TabsTrigger value="arduino">Arduino Example</TabsTrigger>
            <TabsTrigger value="esp32">ESP32 Example</TabsTrigger>
            <TabsTrigger value="library">RoboMetrics Library</TabsTrigger>
          </TabsList>
          <TabsContent value="arduino" className="relative">
            <div className="flex justify-end gap-2 mb-2">
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => copyToClipboard(arduinoExample, "Arduino")}
              >
                <ClipboardCopy className="h-4 w-4 mr-1" /> Copy Code
              </Button>
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => downloadFile(arduinoExample, "RoboMetrics_Arduino.ino", "text/plain")}
              >
                <Download className="h-4 w-4 mr-1" /> Download
              </Button>
            </div>
            <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
              {arduinoExample}
            </pre>
          </TabsContent>
          <TabsContent value="esp32" className="relative">
            <div className="flex justify-end gap-2 mb-2">
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => copyToClipboard(esp32Example, "ESP32")}
              >
                <ClipboardCopy className="h-4 w-4 mr-1" /> Copy Code
              </Button>
              <Button 
                variant="outline" 
                size="sm"
                onClick={() => downloadFile(esp32Example, "RoboMetrics_ESP32.ino", "text/plain")}
              >
                <Download className="h-4 w-4 mr-1" /> Download
              </Button>
            </div>
            <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
              {esp32Example}
            </pre>
          </TabsContent>
          <TabsContent value="library" className="relative space-y-6">
            <div>
              <div className="flex justify-between items-center mb-2">
                <h3 className="text-sm font-semibold">RoboMetrics.h</h3>
                <div className="flex gap-2">
                  <Button 
                    variant="outline" 
                    size="sm"
                    onClick={() => copyToClipboard(libraryHeader, "Header")}
                  >
                    <ClipboardCopy className="h-4 w-4 mr-1" /> Copy
                  </Button>
                  <Button 
                    variant="outline" 
                    size="sm"
                    onClick={() => downloadFile(libraryHeader, "RoboMetrics.h", "text/plain")}
                  >
                    <Download className="h-4 w-4 mr-1" /> Download
                  </Button>
                </div>
              </div>
              <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
                {libraryHeader}
              </pre>
            </div>
            <div>
              <div className="flex justify-between items-center mb-2">
                <h3 className="text-sm font-semibold">RoboMetrics.cpp</h3>
                <div className="flex gap-2">
                  <Button 
                    variant="outline" 
                    size="sm"
                    onClick={() => copyToClipboard(libraryCode, "Implementation")}
                  >
                    <ClipboardCopy className="h-4 w-4 mr-1" /> Copy
                  </Button>
                  <Button 
                    variant="outline" 
                    size="sm"
                    onClick={() => downloadFile(libraryCode, "RoboMetrics.cpp", "text/plain")}
                  >
                    <Download className="h-4 w-4 mr-1" /> Download
                  </Button>
                </div>
              </div>
              <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
                {libraryCode}
              </pre>
            </div>
            <div>
              <div className="flex justify-between items-center mb-2">
                <h3 className="text-sm font-semibold">Example Usage</h3>
                <div className="flex gap-2">
                  <Button 
                    variant="outline" 
                    size="sm"
                    onClick={() => copyToClipboard(simpleExample, "Example")}
                  >
                    <ClipboardCopy className="h-4 w-4 mr-1" /> Copy
                  </Button>
                  <Button 
                    variant="outline" 
                    size="sm"
                    onClick={() => downloadFile(simpleExample, "RoboMetrics_Example.ino", "text/plain")}
                  >
                    <Download className="h-4 w-4 mr-1" /> Download
                  </Button>
                </div>
              </div>
              <pre className="p-4 bg-muted rounded-md overflow-x-auto text-xs">
                {simpleExample}
              </pre>
            </div>
          </TabsContent>
        </Tabs>
      </CardContent>
    </Card>
  );
}
