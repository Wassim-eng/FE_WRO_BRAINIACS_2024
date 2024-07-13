#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <PID_v1.h>
#include "Adafruit_TCS34725.h"

// MPU6050 sensor
MPU6050 mpu;

// Yaw and Gyroscope rate
double yaw = 0; // Raw yaw angle
double gyroZRate = 0; // Gyroscope Z rate

// Gyroscope offset
double gyroZOffset = 0;

// PID parameters
double Kp = 1.8;
double Ki = 0.0;
double Kd = 0.4;
double setpoint = 0.0;
double input, output;

// Servo for steering
Servo steeringServo;

// Motor control pins
const int motorPin1 = 5;
const int motorPin2 = 6;
const int motorSpeed = 80; // Set the speed of the motor (0-255)

// Create the TCS34725 instance
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_16X);

// PID controller
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Variables to handle turning logic
bool turning = false;
double turnStartYaw = 0;
int orangeThreshold=70;
void setup() {
    Wire.begin();
    Serial.begin(9600);
    mpu.initialize();
    calibrateGyro();
    
    steeringServo.attach(11);
    steeringServo.write(93); // Center the servo (assumes 90 degrees is center)

    // Set motor control pins as output
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);

    // Start the motor moving forward
    analogWrite(motorPin1, motorSpeed);
    analogWrite(motorPin2, 0);

    // Initialize PID controller
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-30, 30); // Limit PID output to +/- 30 degrees

    // Initialize the TCS34725 color sensor
    if (tcs.begin()) {
        Serial.println("Found sensor");
    } else {
        Serial.println("No TCS34725 found ... check your connections");
        while (1); // halt!
    }
    tcs.setIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
    tcs.setGain(TCS34725_GAIN_4X);
}

void loop() {
    // Read the MPU6050
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert gyroscope values to rates and apply offsets
    gyroZRate = (gz / 131.0) - gyroZOffset;

    // Calculate the elapsed time
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    double dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // Update yaw using raw gyroscope rate
    yaw += gyroZRate * dt;
    input = yaw;

    // Compute PID output
    myPID.Compute();

    // Constrain the servo output to the 30-degree limit on each side
    int servoOutput = constrain(93 - output, 63, 123);
    steeringServo.write(servoOutput);

    // Read the color sensor
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    float sum = r + g + b;
    float r_ratio = r / sum * 255.0;
    float g_ratio = g / sum * 255.0;
    float b_ratio = b / sum * 255.0;

    // Detect orange color
    if (!turning) {
        if (r_ratio > orangeThreshold && g_ratio > orangeThreshold && b_ratio < orangeThreshold) {
            startTurn(-90); // Turn right
            Serial.println("Orange detected - Turning right");
        }
    }

    // Handle turning logic
    if (turning) {
        if (abs(yaw - turnStartYaw) >= 90) {
            // Turn completed
            turning = false;
            // Reset yaw to zero for the next turn
            yaw = 0;
            setpoint = 0;
            Serial.println("Turn completed - Resetting yaw");
        }
    } else {
        // Normal operation: move forward
        analogWrite(motorPin1, motorSpeed);
        analogWrite(motorPin2, 0);
    }
}

void calibrateGyro() {
    const int numReadings = 1000;
    long gyroZSum = 0;

    Serial.println("Calibrating gyroscope...");

    for (int i = 0; i < numReadings; i++) {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        gyroZSum += gz;
        delay(3);
    }

    gyroZOffset = gyroZSum / (double)numReadings / 131.0;

    Serial.print("Gyro Z Offset: ");
    Serial.println(gyroZOffset);
}

void startTurn(double targetAngle) {
    turnStartYaw = yaw;
    setpoint = yaw + targetAngle;
    turning = true;

    // Stop motor during turn
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 0);
}
