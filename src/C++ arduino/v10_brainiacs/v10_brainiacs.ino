#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Servo.h>
#include <Adafruit_TCS34725.h>

// Initialize MPU6050 and servo
MPU6050 mpu6050(Wire);
Servo myservo;

// Color sensor instance
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_16X);

//juwa limits
int mapped_juwa=0;     
int juwa_lower_limit=0;

// PID constants
double Kp = 1.33;   // Proportional gain
double Ki = 0;      // Integral gain
double Kd = 2;      // Derivative gain

// Variables for PID control
long error = 0;
long lastError = 0;
long de;
long Pr;
int sum = 0;
long integral = 0;

// Target angle
int setpoint = 0;  

// Target angle for lane correction
int waypoint = -82;  

// Counter for lane corrections
int count = 1;       

// Center position for the servo
const int servo_center = 95;  

//Motor speed
const int Motor_Speed=110;

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

    // Start with forward motion
    analogWrite(5, Motor_Speed);  // Assuming pin A for motor control
}

void loop() {
    // Update MPU6050 data
    mpu6050.update();
    error = mpu6050.getAngleZ();
    Serial.print(error);
    // PID control
    error = setpoint - error;
    Pr = error;
    de = error - lastError;
    lastError = error;  
    
    // Update lastError for next iteration
    integral = integral + error;
    sum = Ki * integral + Kp * Pr + Kd * de;

    // Limit sum to prevent excessive servo movement
    if (sum > 30) {
        sum = 30;
    }
    if (sum < -30) {
        sum = -30;
    }
    
    // Adjust servo based on PID output
    myservo.write(-sum + servo_center);  

    // Read color sensor
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    float sum_rgb = r + g + b;
    float r_ratio = r / sum_rgb;
    float g_ratio = g / sum_rgb;
    float b_ratio = b / sum_rgb;

    // Color detection and setpoint adjustment for orange color
    if (r_ratio > 0.35 && g_ratio > 0.25 && b_ratio < 0.32) {
      //Serial.println("ORANGE");
        // Orange color detected
        delay(400);
        analogWrite(5,0);
        int juwa = 0;
        while (juwa >= waypoint * count) {
            mpu6050.update();
            juwa = mpu6050.getAngleZ();
            
            // Map juwa between -83 and 0 to 0 and 30
            // Adjust servo to correct towards right
            myservo.write( servo_center+ 27);
           Serial.println(juwa); 
          analogWrite(5, Motor_Speed);  
        }
        // Update lower limit for juwa
        // Increment lane correction counter  
        setpoint = (waypoint*count)-8;
        count++;     
        // Set new setpoint 
        // Adjust servo after correction
        myservo.write(servo_center); 
        delay(2);
    }

    // Color detection and setpoint adjustment for blue color
    if (b_ratio > 0.35 && r_ratio < 0.32 && g_ratio < 0.35){
         myservo.write(servo_center - 24);
        //Serial.println("BLUE");// Blue color detected
        int juwa = 0;
        while (juwa <= -waypoint * count) {
            mpu6050.update();
            juwa = mpu6050.getAngleZ();
            // Map juwa between -83 and 0 to 0 and 30
            // Adjust servo to correct towards left
            myservo.write(servo_center - 23); 
            // Print mapped juwa for debugging
            Serial.println(mapped_juwa);  
        }
        // Update lower limit for juwa
       // Increment lane correction counter
        count++;
        // Set new setpoint
        setpoint = juwa;  
        // Adjust servo after correction
        myservo.write(servo_center);
        delay(5);  
    }
}
