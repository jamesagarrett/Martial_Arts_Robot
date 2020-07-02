# Martial_Arts_Robot
A maneuverable, robotic machine powered by a Raspberry Pi that uses LIDAR detection to autonomously navigate its environment. This machine is designed and programmed to simulate martial arts sparring for users who want a mobile martial arts training experience when by themselves. Built to support the weight of a punching bag, this machine navigates its surroundings while also keeping track of its user's position, maintaining a close, but safe, distance from them at all times.
## Robot Features
Raspberry Pi 3.0 - Single-board computer used for code execution and debugging  
Slamtec RPLIDAR A1 - 360° distancing detection sensor used for analyzing robot surroundings  
Omni-directional Wheels (3) - Wheels used for in-place turning and 360° movement  
Jaguar Motor Controllers (3) - PWM speed controllers used to specify output speed for the wheels  
Adafruit 16-Channel 12-Bit PWM/Servo Driver - Breakout board connected to the Pi used to output PWM signals to the motor controllers 
## Program Details  
## Credits  
Thank you to Adafruit for providing code to run their PWM/Servo Driver (PCA9685.py) as well as the Slamtec LIDAR Sensor (adafruit_rplidar.py).  
Both programs, which contain their respective licenses, can be found under the "src" folder with all \*.py files.  
These programs can be found at these GitHub links:  
    PCA9685.py: https://github.com/adafruit/Adafruit_Python_PCA9685/tree/master/Adafruit_PCA9685  
    adafruit_rplidar.py: https://github.com/adafruit/rplidar
