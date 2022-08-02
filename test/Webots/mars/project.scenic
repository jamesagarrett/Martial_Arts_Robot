from scenic.simulators.webots.model import WebotsObject

class Human(WebotsObject):
  webotsType: 'HUMAN'
  rotationOffset: 90 deg
  width: 0
  length: 0

class Robot(WebotsObject):
  webotsType: 'MY_ROBOT'
  rotationOffset: 270 deg

monitor terminateOnT:
    for i in range(60):
        wait
    terminate
    

ego = Robot at 0 @ 0
region = PolygonalRegion([(0,0), (-1.3951, -0.5078), 
(-0.62742, -1.34551), (-0.85278, -1.8288), (0.85278, -1.8288),
(0.62742, -1.34551), (1.3951, -0.5078)])
person = Human in region
require (distance to person) > 0.4572
require (person.y < 0)

#record ego.heading as 'robot_heading’
#record (distance to person) as ‘distance'
#record final X as 'blah'
