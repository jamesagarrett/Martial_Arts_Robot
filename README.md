# Martial Arts Robot
## Introduction
A maneuverable, robotic machine that uses LIDAR detection to autonomously navigate its environment. This machine is controlled through a Raspberry Pi and is designed to simulate martial arts sparring for users who want a dynamic training experience when unaccompanied by another human opponent. Built to support the weight of a punching bag, the robot navigates its surroundings while also keeping track of its user's position and maintaining a close but safe distance from them at all times.
## Robot Features
**Raspberry Pi 4.0:** Single-board computer used for code execution  
**Slamtec RPLIDAR A1:** 360° distancing detection sensor used for analyzing robot surroundings  
**Omni-directional Wheels (3):** Specialized wheels used for linear movement in any and all desired directions  
**Jaguar Motor Controllers (3):** PWM speed controllers used to specify output current for the wheels  
**Adafruit 16-Channel 12-Bit PWM/Servo Driver:** Breakout board connected to the Pi used to output PWM signals to the motor controllers 
## Program Details  
This program is split into modules, all contributing to three primary tasks:  
1. Sensor data collection and analysis
2. Optimal angle and distance maneuvering calculations
3. Repositioning of the robot to its new, desired location  

Module Functionality:  
* **PCA9685.py:** Created by Adafruit as a tool for communicating with the 16-Channel PWM Driver
* **Adafruit_rplidar.py:** Created by Adafruit as a tool for retrieving raw data from the Slamtec RPLIDAR sensor  
* **globals.py:** Constant declarations that are used throughout other modules
* **helperFunctions.py:** A single location for frequent data calculations that are utilized in more than one module
* **lidarAnalysis.py:** Contains main() and is where LIDAR sensor data is collected and analyzed to see if the robot should reposition itself
* **maneuverAnalysis.py:** Determines the best angle and distance to move at in order to be a safe distance from the user and any environmental obstacles
* **repositionMachine.py:** Gives output to the wheels, moving the robot to the desired position  
# Video Demos  
A test of the ultrasonic sensors used before switching to a single LIDAR sensor, as the robot approaches an obstacle  
[![Ultrasonic Sensors Test w/ Robot Before Switching to LIDAR](https://img.youtube.com/vi/7fe__-JRM5k/0.jpg)](https://www.youtube.com/watch?v=7fe__-JRM5k "Ultrasonic Sensor Test w/ Robot Before Switching to LIDAR") 

A display of the accuracy and frequency of LIDAR distance measurements, leading to both hardware and software changes from ultrasonic to LIDAR  
[![LIDAR Sensor Test](https://img.youtube.com/vi/xRAZIY07_VE/0.jpg)](https://www.youtube.com/watch?v=xRAZIY07_VE "LIDAR Sensor Test")  

A showcase of the robot reacting to a distance measurement deemed too low, causing the machine's wheel to rotate  
[![LIDAR Test w/ Stationary Robot](https://img.youtube.com/vi/GKys8iLUfHQ/0.jpg)](https://www.youtube.com/watch?v=GKys8iLUfHQ "LIDAR Test w/ Stationary Robot")
## Credits  
Thank you to Adafruit for providing code to run their PWM Driver (PCA9685.py) and the Slamtec LIDAR Sensor (adafruit_rplidar.py).  
Both programs, which contain their respective licenses, can be found under the "src" and "test" folders along with all other .py files.  
They can also be found at their original GitHub links:  
* [PCA9685.py](https://github.com/adafruit/Adafruit_Python_PCA9685/tree/master/Adafruit_PCA9685)    
* [Adafruit_rplidar.py](https://github.com/adafruit/rplidar)

Thank you to the CDC for providing a Product Design Specification document template. Here is the download link for the original template,
as well as where other CDC templates can be found:
* [Download](https://www2a.cdc.gov/cdcup/library/templates/CDC_UP_Product_Design_Template.doc)    
* [CDC Templates](https://www2a.cdc.gov/cdcup/library/templates/default.htm)
