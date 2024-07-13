# Content
- {CAD models} contains all the 3D modeled parts and assembly of the fully-autonomous driving vehicle
- {STL models} contains all the 3D printed parts of traffic signs,parking walls, clamps, covers, etc...
- {T-photos} contains 2 photos of the team (an official one and one funny photo with all team members)
- {V-photos} contains 6 photos of the vehicle (from every side, from top and bottom)
- {Video} contains the video.md file with the link to a video where driving demonstration exists
- {schemes} contains a schematic diagram in form of PNG of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
- {src} contains code of control software for all components which were programmed to participate in the competition
- {models} is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
- {Project Report} contains the report that summarizes the teams work and describes the built prototype.

# Introduction  

In 35 days, Team Brainiacs has developed a self-driving autonomous car for the 2024 WRO future engineers’ competition. This car uses the principles of computer vision and PID to complete the specified rounds of this challenge. The car uses PID to control its movement and steering while using a color sensor to detect the corners of the playfield and turn accordingly. Moreover, computer vision was implemented using a Raspberry Pi 5 and a Logitech C270 camera a to detect the traffic signs and steer the car to complete the whole track.

# Materials used:

- Raspberry Pi 5  
- Logitech C270 camera
- Arduino Uno
- 10000mAh powerbank
- 1 brushless DC motor
- 1 Geared servo motor
- 2500mAh Li-po battery pack
- L298N Dual H-bridge Motor controller
- MPU6050 Gyro sensor
- TCS34725 Color sensor

# Programming Languages Used:

- Arduino C++
- Python

The code for the self-driving autonomous race robotic car consists of several modules that are related to the electromechanical components of the vehicle. The Jetson Nano module runs the computer vision algorithms that analyze camera data to detect obstacles in the car's path. The Arduino Uno module is responsible for controlling the motors, servos, and sensors that enable the car to move and navigate. The MPU6050 gyroscope sensor is used for PID control, which helps to maintain the car's stability and balance while driving. The TCS34725 color sensor detects colors in the environment, which can be used to help the car navigate.

To build and compile the code for the self-driving autonomous race robotic car, the Arduino IDE was used to program the Arduino boards using the C++ language. The Jetson Nano module used Linux and the Python language to perform computer vision tasks. Manual compiling was done using the Linux command prompt on the Jetson Nano. The code for the Arduino boards was compiled into a binary file that could be uploaded to the boards using a USB cable or a wireless connection. The code for the Jetson Nano was written in Python and compiled manually using the Linux command prompt. Once the code was compiled, it could be uploaded to the controllers and executed to control the sensors and actuators and enable the car to drive autonomously. It is important to test the car in a safe environment and make adjustments to the code as needed to improve performance and reliability.
