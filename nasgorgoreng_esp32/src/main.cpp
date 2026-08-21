#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <Servo.h>
#include <ArduinoJson.h>

const char* ssid = "NSGR_LAPTOP";
const char* password = "pieikicok";
const char* rosMachineIpStr = "10.42.0.1";

WiFiUDP udp;
unsigned int localUdpPort = 4210;      // Port we will listen on

IPAddress rosMachineIp;
unsigned int rosMachinePort = 4211; // Must match ROS_LISTEN_PORT

// Servo 1: Steering Motor
Servo steeringServo;
const int SERVO1_PIN = D6;   // GPIO14
// Servo 2: Obstacle Motor
Servo obstacleServo;
const int SERVO2_PIN = D5;   // GPIO12

// Motor
const int MOTOR_IN1  = D3;   // GPIO5
const int MOTOR_IN2  = D4;   // GPIO4
// const int ENABLE_PIN = D2;

// Ultrasonic Sensor
const int TRIG_PIN = D1;
const int ECHO_PIN = D2;
unsigned long lastDistanceSend = 0;
const unsigned long distanceInterval = 100; // send every 100ms

bool   g_newDataAvailable = false;
String g_type = "command";
double g_turnAngle = 0.0;
double g_speed = 0.0;
double g_servoAngle = 90.0;
int    g_moveForXSeconds = 0;

// Buffer for incoming UDP packets
char packetBuffer[255];

void motorStop() {
  // analogWrite(MOTOR_IN1, 0);
  // analogWrite(MOTOR_IN2, 0);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

void motorForward(int speed) {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  // analogWrite(ENABLE_PIN, speed);   // 0–1023
}

void motorReverse(int speed) {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  // analogWrite(ENABLE_PIN, speed);
}

long getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate distance in cm
  return duration * 0.034 / 2;
}

void setup() {
  // Enable built-in LED for status indication
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // Turn on LED (active low)

  // put your setup code here, to run once:
  Serial.begin(115200);

  // Attach servos to their respective pins
  steeringServo.attach(SERVO1_PIN);  // Attach steering servo to pin D5
  steeringServo.write(80);            // Center steering servo
  obstacleServo.attach(SERVO2_PIN);  // Attach obstacle servo to pin D6
  obstacleServo.write(80);            // Center obstacle servo

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  // analogWriteRange(1023); // Set PWM range to 0-1023
  motorStop();

  // Connect to Wi-Fi network
  WiFi.config(
    IPAddress(10, 42, 0, 79),  // Local IP
    IPAddress(10, 42, 0, 1),  // Gateway
    IPAddress(255, 255, 255, 0)  // Subnet
  );
  WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED);

  // Multicast DNS (mDNS)
  // MDNS.begin("nasgorgoreng_bridge");

  // Start listening for UDP packets
  udp.begin(localUdpPort);

  rosMachineIp.fromString(rosMachineIpStr);
} 

void loop() {
  // Task 1: Check for UDP packets (Laptop -> ESP32)
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
      g_type = jsonDoc["type"] | "command";
      g_turnAngle = jsonDoc["turn_angle"] | 0.0;
      g_servoAngle = jsonDoc["servo_angle"] | 90.0;
      g_speed = jsonDoc["speed"] | g_speed; // Retain old speed if not provided
      g_moveForXSeconds = jsonDoc["move_for_x_seconds"] | 0;
      g_newDataAvailable = true;
    }
  }

  // Task 2: If new data is available, update servos and motor
  if (g_newDataAvailable) {
    // command: switch_lane
    // Switch lane by adjusting steering servo
    if (g_type == "switch_lane") {
      int servoAngle = map(g_turnAngle, -90, 90, 165, 15); // Map -30 to 30 degrees to servo range
      servoAngle = constrain(servoAngle, 15, 165);
      steeringServo.write(servoAngle + 135.0); // rotate to right position
      delay(1500); // wait for robot to reach the lane
      steeringServo.write(servoAngle - 135.0); // center back

      return;
    }

    // Update steering servo
    int servoAngle = map(g_turnAngle, -90, 90, 165, 15); // Map -30 to 30 degrees to servo range
    servoAngle = constrain(servoAngle, 15, 165);
    steeringServo.write(servoAngle);

    // Update motor speed and direction
    // int pwm = abs(g_speed) * 2000;
    // pwm = constrain(pwm, 0, 2000);
    // int pwm = map(g_speed, 0, 1, 0, 1023);
    // pwm = constrain(pwm, 0, 255);
    int pwm = (int)(abs(g_speed) * 1023);
    pwm = constrain(pwm, 0, 1023);

    if (g_speed > 0) motorForward(pwm);
    else if (g_speed < 0) motorReverse(pwm);
    else motorStop();

    // Send acknowledgment back to the ROS machine
    JsonDocument ackDoc;
    ackDoc["command"] = "acknowledge";
    ackDoc["ack_data"] = g_speed;
  
    char ackBuffer[256];
    size_t ackLen = serializeJson(ackDoc, ackBuffer);
    udp.beginPacket(rosMachineIp, rosMachinePort);
    udp.write((uint8_t*)ackBuffer, ackLen);
    udp.endPacket();

    g_newDataAvailable = false;
  }

  // Send obstacle distance periodically
  unsigned long now = millis();
  if (now - lastDistanceSend >= distanceInterval) {
    lastDistanceSend = now;

    JsonDocument doc;
    doc["command"] = "telemetry";
    doc["distance"] = getUltrasonicDistance();

    char buffer[128];
    size_t len = serializeJson(doc, buffer);

    udp.beginPacket(rosMachineIp, rosMachinePort);
    udp.write((uint8_t*)buffer, len);
    udp.endPacket();
  }
}
