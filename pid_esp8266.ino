#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// WiFi credentials
const char* ssid = "iQOO Z3 5G";
const char* password = "12345678";

// UDP settings
WiFiUDP udp;
const unsigned int localPort = 12345;
char packetBuffer[255];

// Motor control pins
const int in1 = D1, in2 = D2, pwm1 = D8; // Left Motor
const int in3 = D7, in4 = D4, pwm2 = D3; // Right Motor

// PID Control Parameters
float Kp = 0.5;  // Proportional gain
float Ki = 0;    // Integral gain
float Kd = 0;    // Derivative gain

float previousError = 0;
float integral = 0;
float baseSpeed = 10;  // Base motor speed percentage

unsigned long lastPacketTime = 0;       // Time when last UDP packet was received
const unsigned long timeout = 300;      // 300 milliseconds timeout to stop motors

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected!");

    udp.begin(localPort);
    Serial.printf("UDP server started at port %d\n", localPort);

    // Set motor pins as outputs
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(pwm1, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(pwm2, OUTPUT);
    
    // Set motors to forward direction
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    // Start with 0 speed
    analogWrite(pwm1, 0);
    analogWrite(pwm2, 0);
}

void applyPIDCorrection(float error) {
    float proportional = Kp * error;
    integral += Ki * error;
    float derivative = Kd * (error - previousError);
    previousError = error;

    float correction = proportional + integral + derivative;

    float leftMotorSpeed = baseSpeed - correction;
    float rightMotorSpeed = baseSpeed + correction;

    leftMotorSpeed = constrain(leftMotorSpeed, 0, 100);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 100);

    int pwmLeft = map(leftMotorSpeed, 0, 100, 0, 1023);
    int pwmRight = map(rightMotorSpeed, 0, 100, 0, 1023);

    analogWrite(pwm1, pwmLeft);
    analogWrite(pwm2, pwmRight);

    Serial.printf("Correction Applied - Left Speed: %.2f%%, Right Speed: %.2f%%\n", leftMotorSpeed, rightMotorSpeed);
}

void loop() {
    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(packetBuffer, 255);
        if (len > 0) {
            packetBuffer[len] = '\0';
            float error = atof(packetBuffer);
            applyPIDCorrection(error);

            lastPacketTime = millis(); // Update last packet received time
            Serial.printf("Received packet: %s\n", packetBuffer);
            Serial.printf("Received CTE: %.2f\n", error);
        }
    }

    // Check for timeout and stop motors if no packet received in given time
    if (millis() - lastPacketTime > timeout) {
        analogWrite(pwm1, 0);
        analogWrite(pwm2, 0);
        Serial.println("No packet received - Motors stopped");
    }
}
