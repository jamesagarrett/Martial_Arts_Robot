# Martial_Arts_Robot
A maneuverable, robotic machine powered by a Raspberry Pi that uses LIDAR detection to autonomously navigate its environment. This machine is designed and programmed to simulate martial arts sparring for users who want a mobile martial arts training experience when by themselves. Built to support the weight of a punching bag, this machine navigates its surroundings while also keeping track of its user's position, maintaining a close, but safe, distance from them at all times.
## Robot Features
Raspberry Pi 3.0 - Single-board computer used for code execution and debugging  
Slamtec RPLIDAR A1 - 360° distancing detection sensor used for analyzing robot surroundings  
Omni-directional Wheels (3) - Wheels used for in-place turning and 360° movement  
Jaguar Motor Controllers (3) - PWM speed controllers used to specify output speed for the wheels  
Adafruit 16-Channel 12-Bit PWM/Servo Driver - Breakout board connected to the Pi used to output PWM signals to the motor controllers 
## Program Details  

## Demos
[![Ultrasonic Sensors Test w/ Robot Before Switching to LIDAR](https://img.youtube.com/vi/7fe__-JRM5k/0.jpg)](https://www.youtube.com/watch?v=7fe__-JRM5k "Ultrasonic Sensor Test w/ Robot Before Switching to LIDAR") 

[![LIDAR Sensor Test](https://img.youtube.com/vi/xRAZIY07_VE/0.jpg)](https://www.youtube.com/watch?v=xRAZIY07_VE "LIDAR Sensor Test")  

[![LIDAR Test w/ Stationary Robot](https://img.youtube.com/vi/GKys8iLUfHQ/0.jpg)](https://www.youtube.com/watch?v=GKys8iLUfHQ "LIDAR Test w/ Stationary Robot")
## Credits  
Thank you to Adafruit for providing code to run their PWM Driver (PCA9685.py) and the Slamtec LIDAR Sensor (adafruit_rplidar.py).  
Both programs, which contain their respective licenses, can be found under the "src" folder along with all other \*.py files.  
They can also be found at their original GitHub links:  
* PCA9685.py: https://github.com/adafruit/Adafruit_Python_PCA9685/tree/master/Adafruit_PCA9685    
* Adafruit_rplidar.py: https://github.com/adafruit/rplidar
