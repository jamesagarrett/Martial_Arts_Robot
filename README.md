# Martial Arts Robot
## Introduction
The developed system is a mobile robot that uses LIDAR detection to autonomously navigate its environment. This machine is controlled through a Raspberry Pi and is designed to simulate martial arts sparring for users who want a dynamic training experience when unaccompanied by another human opponent. Built to support the weight of a punching bag, the robot navigates its surroundings while also keeping track of its user's position and maintaining a close but safe distance from them at all times.
## Video Demos  
A test of the robot lidar detection with a single opponent.  
[![Robot Movement Test](https://img.youtube.com/vi/dPjG0p-31MU/0.jpg)](https://youtu.be/dPjG0p-31MU) 

Testing specific features of the machine, including multiple opponents and when it's surrounded on all sides.  
[![Robot Tracking Test](https://img.youtube.com/vi/SVckKlRAG6c/0.jpg)](https://youtu.be/SVckKlRAG6c)
## Robot Features
**Raspberry Pi 4.0:** Single-board computer used for software execution and electronics connectivity  
**Slamtec RPLIDAR A1:** 360° distancing detection sensor used for analyzing robot surroundings  
**Omnidirectional Wheels (3):** Specialized wheels used for linear movement in any and all desired directions  
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
## Credits  
Thank you to Adafruit for providing code to run their PWM Driver (PCA9685.py) and the Slamtec LIDAR Sensor (adafruit_rplidar.py).  
Both programs, which contain their respective licenses, can be found under the /src and /test folders along with all other *.py files.  
They can also be found at their original GitHub links:  
* [PCA9685.py](https://github.com/adafruit/Adafruit_Python_PCA9685/blob/master/LICENSE)      
* [Adafruit_rplidar.py](https://github.com/adafruit/rplidar/blob/master/LICENSE)

Thank you to the CDC for providing a Product Design Specification document template. Here is the download link for the original template,
as well as where other CDC templates can be found:
* [Download](https://www2a.cdc.gov/cdcup/library/templates/CDC_UP_Product_Design_Template.doc)    
* [CDC Templates](https://www2a.cdc.gov/cdcup/library/templates/default.htm)
