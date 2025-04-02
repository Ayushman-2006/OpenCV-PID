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
float Kp = 0.8;  // Proportional gain
float Ki = 0;   // Integral gain
float Kd = 0;   // Derivative gain

float previousError = 0;
float integral = 0;
float baseSpeed = 50;  // Base motor speed percentage

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
    
    // Initialize motors at base speed
    analogWrite(pwm1, map(baseSpeed, 0, 100, 0, 1023));
    analogWrite(pwm2, map(baseSpeed, 0, 100, 0, 1023));
}

// Function to apply PID correction based on cross-track error
void applyPIDCorrection(float error) {
    float proportional = Kp * error;
    integral += Ki * error;
    float derivative = Kd * (error - previousError);
    previousError = error;

    float correction = proportional + integral + derivative;

    // Adjust motor speeds based on correction
    float leftMotorSpeed = baseSpeed - correction;
    float rightMotorSpeed = baseSpeed + correction;

    // Constrain values between 0 and 100%
    leftMotorSpeed = constrain(leftMotorSpeed, 0, 100);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 100);

    int pwmLeft = map(leftMotorSpeed, 0, 100, 0, 1023);
    int pwmRight = map(rightMotorSpeed, 0, 100, 0, 1023);

    // Apply speed control (forward direction only)
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
        }
        Serial.printf("Received packet: %s\n", packetBuffer);

        // Process CTE (received as a float value)
        float error = atof(packetBuffer);
        Serial.printf("Received CTE: %.2f\n", error);
        applyPIDCorrection(error);
    }
}
