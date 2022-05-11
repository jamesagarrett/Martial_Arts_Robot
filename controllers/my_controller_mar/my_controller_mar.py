"""my_controller_mar controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

def run_robot(robot):
    #wheel0_joint
    
    # get the time step of the current world.
    timestep = 32 #int(robot.getBasicTimeStep())
    maxSpeed = 6.28
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getDevice('motorname')
    #  ds = robot.getDevice('dsname')
    #  ds.enable(timestep)
    
    topWheel = robot.getDevice('wheel0_joint')
    leftWheel = robot.getDevice('wheel1_joint')
    rightWheel = robot.getDevice('wheel2_joint')
    
    topWheel.setPosition(float('inf'))
    leftWheel.setPosition(float('inf'))
    rightWheel.setPosition(float('inf'))
    
    topWheel.setVelocity(0.0)
    leftWheel.setVelocity(0.0)
    rightWheel.setVelocity(0.0)
    
    # sensor = robot.getDevice('lidar1')
    # sensor.enable(timestep)
    # sensor.enablePointCloud()
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
        # distances = sensor.getRangeImage()
        # print(distances[::-1])
        
        # Process sensor data here.
    
        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)
        angle = math.radians(float(90))
        w1 = 2 * math.cos(angle) / 3
        w2 = (math.cos(angle) - math.sqrt(3) * math.sin(angle)) / -3
        w3 = (math.cos(angle) + math.sqrt(3) * math.sin(angle)) / -3
        print(w1,w2,w3)
        topWheel.setVelocity(maxSpeed*0)
        leftWheel.setVelocity(maxSpeed*w2)
        rightWheel.setVelocity(maxSpeed*w3)
    
    # Enter here exit cleanup code.
    
if __name__ == "__main__":
# create the Robot instance.
    robot = Robot()
    run_robot(robot)
