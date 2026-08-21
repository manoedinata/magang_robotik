// ESP32/ESP8266 Wi-Fi UDP Bridge Server
// 1. Receives UDP (from Laptop) -> Forwards to SoftwareSerial (to STM32)
// 2. Receives SoftwareSerial (from STM32) -> Forwards via UDP (to Laptop)

#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#elif defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#endif

#include <WiFiUdp.h>
#include <ArduinoJson.h>

#include "pins.h"
#include "wifi.h"

// --- Networking Objects ---
WiFiUDP udp;
IPAddress rosMachineIp;
unsigned int localUdpPort = 4210;      // Port we will listen on
unsigned int rosMachinePort = 4211; // Port to send data to ROS machine

// --- Global variables for data sharing ---
// (No 'volatile' needed as it's all in the main loop)
bool   g_newDataAvailable = false;
double g_turnAngle = 0.0;
double g_speed = 0.0;

// Buffer for incoming UDP packets
char packetBuffer[255];

// Buffer for incoming Serial data from STM32
#define STM_BUFF_SIZE 64
char stmBuffer[STM_BUFF_SIZE];
int stmBufferPos = 0;

void setup() {
  // Enable built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize Serial connection to STM32 for debugging
  // This uses the main Serial port (GPIO1=TX, GPIO3=RX)
  // so we can't debug via USB.
  Serial.begin(115200);
  while (!Serial);

  // Start Wi-Fi hotspot
  WiFi.softAP(ssid, password);

  // Start listening for UDP packets
  udp.begin(localUdpPort);
  
  rosMachineIp.fromString("192.168.4.27");
}

void loop() {

  // Task 1: Check for UDP packets (Laptop -> STM32)
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = '\0';
    }

    // --- Parse the JSON ---
    JsonDocument jsonDoc;
    DeserializationError error = deserializeJson(jsonDoc, packetBuffer);
    if (!error) {
      g_turnAngle = jsonDoc["turn_angle"] | 0.0;
      g_speed = jsonDoc["speed"] | g_speed; // Retain old speed if not provided
      g_newDataAvailable = true;
    }
  }


  // Task 2: Check for Serial data (STM32 -> Laptop) NON-BLOCKING
  while (Serial.available() > 0) {
    char inChar = Serial.read();

    // Check for newline character
    if (inChar == '\n') {
      stmBuffer[stmBufferPos] = '\0'; // Null-terminate the string

      // We have a complete line, process it
      if (stmBufferPos > 0) {
        String data = String(stmBuffer); // Convert buffer to String for parsing
        data.trim();

        int colonIndex = data.indexOf(':');
        if (colonIndex != -1) { 
          // Parse Data
          float obstacle_distance = data.substring(colonIndex + 1).toFloat();

          // Build JSON
          JsonDocument jsonDoc;
          jsonDoc["obstacle_distance"] = obstacle_distance;
          jsonDoc["timestamp"] = millis();

          // Send to Laptop (memory-safe way)
          udp.beginPacket(rosMachineIp, rosMachinePort);
          serializeJson(jsonDoc, udp);
          udp.endPacket();
        }
      }
      // Reset buffer position for the next line
      stmBufferPos = 0;

    } else if (stmBufferPos < (STM_BUFF_SIZE - 1)) {
      // Add char to buffer if it's not a newline and there's space
      if (inChar != '\r') { // Ignore carriage return
        stmBuffer[stmBufferPos] = inChar;
        stmBufferPos++;
      }
    } else {
      // Buffer overflow! Discard the buffer.
      stmBufferPos = 0;
    }
  } // end while(Serial.available)


  // Task 3: Send new data to STM32 (Laptop -> STM32)
  if (g_newDataAvailable) {
    g_newDataAvailable = false; // Clear the flag immediately

    // Speed to PWM conversion
    double pwmValue = g_speed * 1000;

    // Angle to PWM conversion
    long turnPWM = map(g_turnAngle, -90, 90, 1000, 2000);
    turnPWM = constrain(turnPWM, 1000, 2000);

    // Build the command string to send to STM32
    String stmData = "S:1500,T:" + String(turnPWM) + ",M:" + String((long)pwmValue) + "\n";

    Serial.print(stmData); // Send to STM32

    // NO DELAY NEEDED HERE!
    // The g_newDataAvailable flag already prevents spamming.
  }
}
