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

The self-driving race car's code involves several key modules. A Raspberry Pi 5 runs computer vision algorithms to detect obstacles using camera data. Arduino Uno modules control the car's motors, servos, and sensors for movement and navigation. An MPU6050 gyroscope sensor ensures the car stays centered and stable while driving. Additionally, a TCS34725 color sensor detects environmental colors to aid in navigation and turning.

Our team’s work was based on the popular saying “ one hand can’t clap alone”. With the coach’s help and monitor the members could perfectly work by distributing work. Together Hassan and Mahmoud worked on the connection diagram and code writing. For processing the computer vision to detect the traffic signs, the coach was able to teach  the members how to be working in such an advanced level. As a result of this great work, the brainiacs team perfectly managed to perform a self-driving robot applicable with all rules provided for the competition.  

# Materials used:

•	Raspberry Pi 5 8GB Arduino UNO R3
•	GY-87 gyroscope
•	Adafruit TCS34725 color sensor
•	Logitech C270 720p30 webcam
•	1 High Speed DC motor
•	1 Mg995 geared servo motor
•	4x 4800mAh Lithium-Ion battery 18650 sticks
•	L298N Dual H-bridge Motor controller
•	Auxiliary 3D printed parts and laser cut chassis
•	Power bank 10000mAh
•	Switch module 
•	Tactile Push Button
•	LM2596S Step-down buck converter


# Programming Languages Used:

- Arduino C++
- Python

The code for the self-driving autonomous race robotic car consists of several modules that are related to the electromechanical components of the vehicle. The Raspberry Pi 5 module runs the computer vision algorithms that analyze camera data to detect obstacles in the car's path. The Arduino Uno module is responsible for controlling the motors, servos, and sensors that enable the car to move and navigate. The MPU6050 gyroscope sensor is used for PID control, which helps to maintain the car's stability and balance while driving. The TCS34725 color sensor detects colors in the environment, which can be used to help the car navigate.

To build and compile the code for the self-driving autonomous race robotic car, the Arduino IDE was used to program the Arduino boards using the C++ language. The Raspberry Pi 5 module used Linux-debian based operating system and the Python language to perform computer vision tasks. Manual compiling was done using the Linux command prompt on the Raspberry Pi. The code for the Arduino board was compiled into a binary file that could be uploaded to the boards using a USB cable or a wireless connection. The code for the Pi was written in Python and compiled manually using the Linux command prompt. Once the code was compiled, it could be uploaded to the controllers and executed to control the sensors and actuators and enable the car to drive autonomously. It is important to test the car in a safe environment and make adjustments to the code as needed to improve performance and reliability.

This self-driving car was established to be moving on a random lane according to the judges’ choice. The car must complete three rounds, an open round where it must complete 3 perfect turns on the lane chosen by the judge, the car during its turns would detect colored stripes on the 90 degree turn to either turn left (detecting orange color) where it moves clockwise or turn to the right (detecting blue color) where it moves in a counter clockwise motion. This challenge has a 3-minute max. time duration for the car to cover the whole 3 turns.

In the second challenge obstacles must be detected and avoided by the car. The car must detect traffic signs and avoid them. The self-driving car must pass the red pillar from the right and the green pillar from the left without moving or knocking down the obstacles outside the circular border drawn around them. 

The self-driving car must also perform a parallel parking in a lot 1.25 times the size of the car.   

The project by performing these challenges in a small scale and passing them in  a perfect manner could be performed in bigger scales to make all vehicles in the market as in our project. Develop an autonomous vehicle capable of navigating urban environments to perform all human purposes from transportation to delivery. Such an idea is performed using computer vision for obstacle detection using raspberry pi 5 and Arduino scripts to analyze and compute data to perform various stability and motion control through the different sensors, actuators, and devices.
