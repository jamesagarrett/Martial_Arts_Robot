##	
##	James Garrett
##
##	Martial_Arts_Robot 
##  Last Updated: July 6, 2021
##
##	sensorAnalysis.py
##  Last Updated: July 6, 2021
##
##	Collect and analyze sensor distance data to determine whether repositioning 
##	is needed. If so, also determine in what manner the machine needs to 
##	maneuver itself to be back within the desired range described in globals.py.
##

from math import atan, ceil, degrees, floor, round
from bisect import bisect_left	
from os import system

##Module that includes functions that are used throughout the project.
from helperFunctions import getCollinearDistance, getCartesianAngle

##Module to determine how the machine will reposition itself toward or
##away from the opponent.
from maneuverAnalysis import calculateObjMovement, calculateOppMovement

##Module for performing repositioning of the machine, based on sensor data 
##collection and analysis.
from repositionMachine import rotateMachine

##THESE VALUES ARE NOT TO BE CHANGED!
from globals import DES_OPP_ANGLE,\
					FRONT_ANGLE_MAX,\
					FRONT_ANGLE_MIN,\
					LEFT_TURN_ANGLE_MAX,\
					LEFT_TURN_ANGLE_MIN,\
					MACH_RADIUS,\
					PWM_FREQ,\
					PWM_PORTS,\
					RIGHT_TURN_ANGLE_MAX,\
					RIGHT_TURN_ANGLE_MIN,\
					SENSOR,\
					SNS_MAX_DISTANCE,\
					SNS_MIN_DISTANCE,\
					SNS_OPP_DISTANCE,\
					START_TICK,\
					STOP_SPEED,\
					WHEELS

#--------------------------------------  --------------------------------------#
#--------------------------------------  --------------------------------------#

#########################################
##
##	FUNCTION DEFINITION
##
#########################################

## ********************************************************
## name:	  collectData
## called by: sensorAnalysis.main()
## passed:	  nothing
## returns:   nothing
## calls:	  sensorAnalysis.interpretData()
##			  helperFunctions.getCartesianAngle()
##
## Retrieve sensor readings and store all angles and	  *
## associated distances.								  *
## ********************************************************
def collectData():

	#####################################
	##
	##	VARIABLE DECLARATION
	##
	#####################################

	sensorDistances = [0.0]*360 ##A list of all measured sensor distances for 
								##each angle, 0-359.

	distancesCount = 0			##The amount of measured sensor distances from
								##the current iteration of the distance 
								##collection loop. 

	prevDistCount = 0			##The value of distancesCount from the previous
								##iteration of the distance collection loop.

	listStartPnt = 0			##The position in sensorDistances to begin 
								##iterating when manually entering values; done
								##when the sensor cannot read a value by itself.
	
	#####################################

	for scan in SENSOR.iter_scans():

		prevDistCount = distancesCount

		##Collect and store sensor data. Distances are stored in such a way
		##that the corresponding index for each value represents its angle on
		##the standard Cartesian plane. Due to the clockwise rotation of the 
		##sensor, values are initially read in this same manner before being 
		##adjusted here. Distances are converted from millimeters to inches.
		for (_, angle, distance) in scan:
			angle = getCartesianAngle(round(angle))

			if(sensorDistances[angle] == 0.0 and distance > 0.0):
				sensorDistances[angle] = 0.0393 * distance
				distancesCount += 1

		##Cease data collection if all angles have an accompanying distance 
		##value. If no distance can be found for an angle, set its distance 
		##equal to the value of the angle before it. This may occur do to 
		##unpredictable environmental factors, such as excessive lighting. 
		if(distancesCount >= 360):
			break
		
		elif(distancesCount == prevDistCount):			  
			for x in range (-359, 1):
				if(sensorDistances[x-1] == 0.0 and sensorDistances[x] > 0.0):
					listStartPnt = x + 1
					break
	
			for x in range (listStartPnt, listStartPnt + 360):
				if(sensorDistances[x] == 0.0):
					sensorDistances[x] = sensorDistances[x-1]

			break
		
	##Prevents adafruit_rplidar.py runtime error when attempting to collect 
	##data again
	SENSOR.stop()
	SENSOR.disconnect()
	SENSOR.connect()

	#print(distancesCount, "Distances Found\n")
	interpretData(sensorDistances)

	return

## ********************************************************
## name:	  interpretData
## called by: sensorAnalysis.collectData()
## passed:	  float[] sensorDistances 
## returns:   nothing
## calls:	  helperFunctions.getCollinearDistance()
##			  maneuverAnalysis.calculateObjMovement(),
##							   calculateOppMovement()
##			  repositionMachine.rotateMachine()
##
## Analyze sensor distance data and determine the		  *
## appropriate course of action for maneuvering the		  *
## machine if applicable.								  *
## ********************************************************
def interpretData(distanceValues):

	#####################################
	##
	##	VARIABLE DECLARATION
	##
	#####################################
							
	activeDistances = []		##Distances recorded to be outside the desired 
								##range of the machine; that is, a value in 
								##front of the machine > MAX_DISTANCE or a 
								##value recorded anywhere < MIN_DISTANCE.

	activeAngles = []			##The corresponding angles associated with 
								##each activeDistances value, or in the case of
								##turning toward the opponent, the list of 
								##angles where the opponent is currently 
								##located.

	insertPoint = 0				##The position in which an item is added to the
								##activeAngles and activeDistances lists.

	wallAnglesCount = 0			##The amount of adjacent angles on either side
								##of the current value being added to 
								##activeAngles. These angles will also be added
								##to activeAngles if they are detecting the same
								##object as the original angle.

	adjustedDistance = 0		##The returned value of getCollinearDistance()
	adjustedDistance2 = 0

	tooClose = False			##Set to True if the machine detects an 
								##object that is too close.

	opponentSpan = 0			##The amount of angles that have corresponding
								##distance values matching the location of the
								##opponent.
								
	opponentAngle = 0			##The average of all angles that are reading the
								##opponent.

	tooFar = False				##Set to True if the machine detects the 
								##opponent is too far away.

	opponentFound = False		##Set to True if the machine detects the 
								##opponent in front and tooClose == tooFar == 
								##False.

	turningCW = False			##Set to True if the machine detects the 
								##opponent too far left of center.

	turningCCW = False			##Set to True if the machine detects the 
								##opponent too far right of center. 

	#####################################

	##Collect position data for objects if considered too close to the machine.
	##getCollinearDistance() is used iff there is first an object detected 
	##within SNS_MIN_DISTANCE, and looks for other distance values that lie on 
	##the same line as the original, out-of-range value. This will better allow 
	##for detection of walls and other similar objects.
	for x in range (0, 360):

		if(MACH_RADIUS < distanceValues[x] < SNS_MIN_DISTANCE):
			insertPoint = bisect_left(activeAngles, x)

			if(len(activeAngles) == 0 or insertPoint == 
			len(activeAngles) or x != activeAngles[insertPoint]):
			   activeAngles.insert(insertPoint, x)
			   activeDistances.insert(insertPoint, distanceValues[x])
			else:
				continue
			
			wallAnglesCount = ceil(degrees(atan(MACH_RADIUS/distanceValues[x])))

			for y in range (x - wallAnglesCount, x + wallAnglesCount + 1):
				if(y < 0):
					y += 360
				elif(y > 359): 
					y -= 360

				adjustedDistance = getCollinearDistance(y, x, distanceValues[x])

				if(MACH_RADIUS < distanceValues[y] <= adjustedDistance):
					
					insertPoint = bisect_left(activeAngles, y)
					
					if(len(activeAngles) == 0 or insertPoint == 
					len(activeAngles) or y != activeAngles[insertPoint]):
						activeAngles.insert(insertPoint, y)
						activeDistances.insert(insertPoint, distanceValues[y])

			tooClose = True

	if(tooClose):
		##The opponent's location is needed to make sure that when moving away
		##from objects that are currently too close, the machine is not in turn
		##too far from the opponent after moving.
		for x in range (FRONT_ANGLE_MIN, FRONT_ANGLE_MAX + 1):
			adjustedDistance = getCollinearDistance(x, DES_OPP_ANGLE, 
													SNS_MAX_DISTANCE)

			if(MACH_RADIUS < distanceValues[x] <= adjustedDistance):
				opponentAngle += x
				opponentSpan += 1
		
		if(opponentSpan > 0):
			opponentAngle = floor(opponentAngle/opponentSpan)
		else:
			opponentAngle = DES_OPP_ANGLE

		print("Status: Too Close\n")
		#calculateObjMovement(activeAngles, activeDistances, opponentAngle, 
		#					  distanceValues)

		return

	##Barring any objects too close to the machine, look for the opponent being
	##too far away. getCollinearDistance() is used here as to allow the opponent
	##to move laterally when at MAX_DISTANCE, without the machine detecting them
	##as too far.
	for x in range (FRONT_ANGLE_MIN, FRONT_ANGLE_MAX + 1):
		adjustedDistance = getCollinearDistance(x, DES_OPP_ANGLE, 
												SNS_MAX_DISTANCE)

		if(MACH_RADIUS < distanceValues[x] <= adjustedDistance):
			activeAngles.clear()
			activeDistances.clear()
			tooFar = False
			opponentFound = True
			break

		adjustedDistance = getCollinearDistance(x, DES_OPP_ANGLE,
												 SNS_OPP_DISTANCE)

		if(distanceValues[x] <= adjustedDistance):
			activeAngles.append(x)
			activeDistances.append(distanceValues[x])	
			tooFar = True	 

	if(tooFar):
		print("Status: Too Far\n")
		calculateOppMovement(activeAngles, activeDistances, distanceValues)

		return

	##Analyze turning only after determining nothing is too close or too far 
	##from the machine.
	if(not opponentFound):
		for x in range (LEFT_TURN_ANGLE_MIN, LEFT_TURN_ANGLE_MAX + 1):
			y = ceil((LEFT_TURN_ANGLE_MIN + LEFT_TURN_ANGLE_MAX) / 2)

			adjustedDistance = getCollinearDistance(x, y, SNS_MAX_DISTANCE)

			if(SNS_MIN_DISTANCE <= distanceValues[x] <= adjustedDistance):
				activeAngles.append(x)
				turningCCW = True
			
		if(not turningCCW):
			for x in range (RIGHT_TURN_ANGLE_MIN, RIGHT_TURN_ANGLE_MAX + 1):
				y = ceil((RIGHT_TURN_ANGLE_MIN + RIGHT_TURN_ANGLE_MAX) / 2)
				
				adjustedDistance = getCollinearDistance(x, y, SNS_MAX_DISTANCE)
				
				if(SNS_MIN_DISTANCE <= distanceValues[x] <= adjustedDistance):
					activeAngles.append(x)
					turningCW = True

		if(turningCCW or turningCW):
			print("Status: Not Centered")
			print(activeAngles)
			print("CCW" if turningCCW else "CW\n")
			opponentSpan = len(activeAngles)
			rotateMachine(turningCW, opponentSpan)
		else:
			print("Status: No Opponent\n")
	else:
		print("Status: Good\n")

	return

#--------------------------------------  --------------------------------------#
#--------------------------------------  --------------------------------------#

def main():
	start = input("PRESS <ENTER> TO BEGIN")
	clear = lambda: system('clear')

	try:
		reset = 0;

		while(True):
			#if(reset % 10 == 0):
			#	clear()
			#	print("RUNNING\n")
			#reset += 1

			#collectData()

			sensorDistances = [MACH_RADIUS + 30.0]*360
			for x in range (FRONT_ANGLE_MIN, FRONT_ANGLE_MAX+1):
				sensorDistances[x] = SNS_OPP_DISTANCE + 12

			for x in range (265, 276):
				sensorDistances[x] = SNS_MAX_DISTANCE + 3
			
			interpretData(sensorDistances)
			break

	except KeyboardInterrupt:
		clear()
		print("TERMINATING")
		WHEELS.set_pwm_freq(PWM_FREQ)
		WHEELS.set_pwm(PWM_PORTS[0], START_TICK, STOP_SPEED)
		WHEELS.set_pwm(PWM_PORTS[1], START_TICK, STOP_SPEED)
		WHEELS.set_pwm(PWM_PORTS[2], START_TICK, STOP_SPEED)
		SENSOR.stop()
		SENSOR.disconnect()

	return

main()

