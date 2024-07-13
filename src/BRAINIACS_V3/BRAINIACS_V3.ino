#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include "Adafruit_TCS34725.h"

// MPU6050 sensor
MPU6050 mpu;

// Complementary filter parameters
double yaw = 0; // Filtered yaw angle
double gyroZRate = 0; // Gyroscope Z rate

// Gyroscope offset
double gyroZOffset = 0;

// Kalman filter parameters
double kalmanYaw = 0;
double P[2][2] = {{1, 0}, {0, 1}};
double Q_angle = 0.001;
double Q_bias = 0.003;
double R_measure = 0.03;
double angle = 0;
double bias = 0;
double rate = 0;

// PID parameters
double Kp = 3;
double Ki = 0.0;
double Kd = 0.2;
double previousError = 0;
double integral = 0;

// Servo for steering
Servo steeringServo;

// Motor control pins
const int motorPin1 = 5;
const int motorPin2 = 6;
const int motorSpeed = 30; // Set the speed of the motor (0-255)

// Create the TCS34725 instance
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_16X);

// Thresholds for differentiating colors
int blueThreshold = 100;
int orangeThreshold = 70;

// Yaw target for turns
double yawTarget = 0;
double setpoint = 0.0;

// State variables
bool turnInProgress = false;
bool straightening = false;

void setup() {
    Wire.begin();
    Serial.begin(9600);
    
    // Initialize MPU6050 sensor
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1);
    }
    
    // Calibrate the gyroscope
    calibrateGyro();
    
    // Attach the steering servo to pin 10
    steeringServo.attach(10);
    steeringServo.write(94); // Center the servo (assumes 90 degrees is center)
    
    // Set motor control pins as output
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    
    // Start the motor moving forward
    analogWrite(motorPin1, motorSpeed);
    analogWrite(motorPin2, 0);
    
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

    // Complementary filter for yaw
    yaw += gyroZRate * dt;
    
    // Kalman filter for yaw
    kalmanYaw = kalmanFilter(yaw, gyroZRate, dt);
    
    // Read the color sensor
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    float sum = r + g + b;
    float r_ratio = r / sum * 255.0;
    float g_ratio = g / sum * 255.0;
    float b_ratio = b / sum * 255.0;

    // Determine color and set yaw target
    if (!turnInProgress) {
        if (b_ratio > blueThreshold && r_ratio < blueThreshold && g_ratio < blueThreshold) {
            startTurn(90); // Turn left
            Serial.println("Blue detected - Turning left");
        } else if (r_ratio > orangeThreshold && g_ratio > orangeThreshold && b_ratio < orangeThreshold) {
            startTurn(-90); // Turn right
            Serial.println("Orange detected - Turning right");
        }
    }

    // PID control for steering only when moving forward and not turning
    if (!turnInProgress || straightening) {
        double error = setpoint - kalmanYaw;
        integral += error * dt;
        double derivative = (error - previousError) / dt;
        double output = Kp * error + Ki * integral + Kd * derivative;
        previousError = error;
        
        // Invert the output to switch the servo direction
        output = -output;
        
        // Constrain the servo output to the 30-degree limit on each side
        int servoOutput = constrain(94 + output, 66, 122);
        steeringServo.write(servoOutput);
        
        // Print the current angle and servo output
        Serial.print("Filtered Yaw: ");
        Serial.print(kalmanYaw);
        Serial.print(" | Servo Output: ");
        Serial.println(servoOutput);
        
        // Check if straightening is needed
        if (straightening && abs(kalmanYaw - setpoint) < 2.0) {
            straightening = false;  // Reset straightening flag
        }
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

double kalmanFilter(double newAngle, double newRate, double dt) {
    rate = newRate - bias;
    angle += dt * rate;
    
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;
    
    double S = P[0][0] + R_measure;
    double K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
    
    double y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;
    
    double P00_temp = P[0][0];
    double P01_temp = P[0][1];
    
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
    
    return angle;
}

// Function to initiate a turn
void startTurn(double targetAngle) {
    yawTarget = targetAngle;
    setpoint = kalmanYaw + targetAngle;
    turnInProgress = true;
    
    // Stop PID output during turn
    integral = 0;
    straightening = true;  // Set flag to straighten after turn
}
