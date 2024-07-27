#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Servo.h>
#include <Adafruit_TCS34725.h>

// Initialize MPU6050 and servo
MPU6050 mpu6050(Wire);
Servo myservo;

// Color sensor instance
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_16X);

// PID constants
double Kp = 1.3;   // Proportional gain
double Ki = 0;     // Integral gain
double Kd = 0.5;   // Derivative gain

// Variables for PID control
long error = 0;
long lastError = 0;
long de;
long Pr;
int sum = 0;
long integral = 0;

// Target angle
int setpoint = 0;

// Deadband range
const int deadband = 0.5;  // Example deadband range

// Target angle for lane correction
int waypoint = -165;

// Counter for lane corrections
int count = 1;

// Center position for the servo
const int servo_center = 92;

// Motor speed
const int Motor_Speed = 80;

// Servo movement limits
const int servo_min = 60;
const int servo_max = 120;

void setup() {
  Serial.begin(9600);

  // Initialize MPU6050 and servo
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  // Attach servo to pin 11
  myservo.attach(11);

  // Initialize color sensor
  if (tcs.begin()) {
    Serial.println("Color sensor found");
  } else {
    Serial.println("Color sensor not found. Check wiring.");
    while (1);  // Halt if sensor not found
  }
  mpu6050.update();
  setpoint = mpu6050.getAngleZ();
}

void loop() {
  if (count<=12){
  // Start with forward motion
  analogWrite(5, Motor_Speed);  // Assuming pin A for motor control

  // Update MPU6050 data
  mpu6050.update();
  error = mpu6050.getAngleZ();

  // PID control with deadband
  error = setpoint - error;
  
 

  Pr = error;
  de = error - lastError;
  lastError = error;
  // Update lastError for next iteration
  integral = integral + error;
  sum = Ki * integral + Kp * Pr + Kd * de;
  // Adjust servo based on PID output and constrain the final position
  int servo_position = constrain(-sum + servo_center, servo_min, servo_max);
  myservo.write(servo_position);

  // Read color sensor
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  float sum_rgb = r + g + b;
  float r_ratio = r / sum_rgb;
  float g_ratio = g / sum_rgb;
  float b_ratio = b / sum_rgb;

  // Color detection and setpoint adjustment for orange color
  if (r_ratio > 0.35 && g_ratio > 0.25 && b_ratio < 0.32) {
    int juwa = 0;
    while (juwa >= waypoint * count) {
      mpu6050.update();
      juwa = mpu6050.getAngleZ();
      // Adjust servo to correct towards right
      myservo.write(constrain(servo_center + 29, servo_min, servo_max));
    }
  
    count++;
    setpoint = juwa;
    myservo.write(servo_center);
  
  }
  
    // Color detection and setpoint adjustment for blue color
    if (b_ratio > 0.35 && r_ratio < 0.32 && g_ratio < 0.35){
        int juwa = 0;
        while (juwa <= -waypoint * count) {
            mpu6050.update();
            juwa = mpu6050.getAngleZ();
            // Adjust servo to correct towards left
          myservo.write(constrain(servo_center - 29, servo_min, servo_max));
   }
    count++;
    setpoint = juwa;
    myservo.write(servo_center);
   }
  }
  else{
  analogWrite(5, 0);  // Assuming pin A for motor control
  
  }
}
