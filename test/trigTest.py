from math import acos, atan, ceil, cos, degrees, floor, radians, sin, sqrt, tan

def findMoveDistance(objAngles, objDistances, maneuverAngle, desiredDistance):

    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################

    compMoveAngle = 0           ##The complement of the moveAngle value, that
                                ##is, the angle 180 degrees from moveAngle.

    trigVars = [0.0]*4          ##Variables used to determine wheelSpeeds
                                ##values.

    a = b = c = 0.0             ##The 3 coefficients needed for the quadratic 
                                ##formula; used to calculate maneuverDistance.

    quadAnswer = 0              ##The solution coming from solving the quadratic
                                ##equation.

    maneuverDistance = 0.0      ##The distance the machine will travel when 
                                ##repositioning.

    closestObjAngle = 0         ##The objAngle value that results in the largest
                                ##maneuverDistance value.

    #####################################
    
    ##In order to calculate the value of maneuverDistance, we must know the 
    ##smallest distance between the machine and any given object that is too 
    ##close. The amount of movement required to reach the desiredDistance away
    ##from this object is our maneuverDistance.
     
    if(maneuverAngle < 180):
        compMoveAngle = maneuverAngle + 180
    else:
        compMoveAngle = maneuverAngle - 180

    ##See documentation for explanation on how the following equations were 
    ##determined.
    trigVars[0] = cos(radians(compMoveAngle))
    trigVars[1] = sin(radians(compMoveAngle))
    
    ##Variable "a" for the quadratic formula is equal to: trigVars[0]**2 + 
    ##trigVars[1]**2. Using trigonometric identities, this means a = 1.0.
    a = 1.0 

    for angle, distance in zip(objAngles, objDistances):
        trigVars[2] = cos(radians(angle)) * distance
        trigVars[3] = sin(radians(angle)) * distance

        b = 2.0 * trigVars[0] * trigVars[2] +  2.0 * trigVars[1] * trigVars[3]
        c = -(desiredDistance**2 - trigVars[2]**2 - trigVars[3]**2)

        #We can remove "a" from the equation due to its value being 1.0.
        quadAnswer = (-b + sqrt(b**2 - 4*c))/2

        if(quadAnswer > 0):
            print(quadAnswer, angle, distance)
            if(quadAnswer > maneuverDistance):
                maneuverDistance = quadAnswer
                closestObjAngle = angle
        else:
            quadAnswer = (-b - sqrt(b**2 - 4*c))/2
            print(quadAnswer, angle, distance)
            if(quadAnswer > maneuverDistance):
                maneuverDistance = quadAnswer
                closestObjAngle = angle

    return maneuverDistance, closestObjAngle

def findMoveDistance2(objAngles, objDistances, maneuverAngle, desiredDistance):

    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################
    
    maneuverDistance = 0.0      ##The distance the machine will travel when 
                                ##repositioning.

    closestObjAngle = 0         ##The objAngle value that results in the largest
                                ##maneuverDistance value.

    objXComp = 0.0              ##The x(horizontal) component of an object's 
                                ##location relative to the machine.

    objYComp = 0.0              ##The y(vertical) component of an object's 
                                ##location relative to the machine.

    maneuverXComp = 0.0         ##The x(horizontal) component of the 
                                ##maneuverDistance value.

    maneuverYComp = 0.0         ##The y(vertical) component of the 
                                ##maneuverDistance value.

    a = b = c = 0.0             ##The 3 coefficients needed for the quadratic 
                                ##formula; used to calculate maneuverXComp.

    #####################################
    
    ##In order to calculate the value of maneuverDistance, we must know the 
    ##smallest distance between the machine and any given object that is too 
    ##close. The amount of movement required to reach the desiredDistance away
    ##from this object is our maneuverDistance.
        
    for x in range (len(objAngles)):

        objXComp = cos(radians(objAngles[x])) * objDistances[x]
        objYComp = sin(radians(objAngles[x])) * objDistances[x]

        if(maneuverAngle == 90 or maneuverAngle == 270):
            maneuverXComp = 0.0
            maneuverYComp = sqrt(desiredDistance**2 - objXComp**2) \
                            - abs(objYComp)
            print(maneuverYComp, objAngles[x], objDistances[x])
            if(maneuverYComp > maneuverDistance):
                maneuverDistance = maneuverYComp
                closestObjAngle = objAngles[x]
        else:
            a = tan(radians(maneuverAngle))**2 + 1.0
            b = -2*objXComp - 2*objYComp*tan(radians(maneuverAngle))
            c = objXComp**2 + objYComp**2 - desiredDistance**2

            maneuverXComp = (-b + sqrt(b**2 - 4*a*c)) / (2*a)           

            ##Check if maneuverXComp is in the correct quadrant for the value 
            ##of maneuverAngle.
            if((maneuverXComp < 0 and 90 < maneuverAngle < 270) 
                    or (maneuverXComp > 0 and (0 <= maneuverAngle < 90 
                                               or 270 < maneuverAngle < 360))):
                maneuverYComp = maneuverXComp * tan(radians(maneuverAngle))
                print(sqrt(maneuverXComp**2 + maneuverYComp**2), objAngles[x], objDistances[x])
                if(sqrt(maneuverXComp**2 + maneuverYComp**2) 
                        > maneuverDistance):
                    maneuverDistance = sqrt(maneuverXComp**2 + maneuverYComp**2)
                    closestObjAngle = objAngles[x]

            #Recalculate maneuverXComp if in the incorrect quadrant.
            elif((maneuverXComp > 0 and 90 < maneuverAngle < 270) 
                    or (maneuverXComp < 0 and (0 <= maneuverAngle < 90 
                                               or 270 < maneuverAngle < 360))):
                maneuverXComp = (-b - sqrt(b**2 - 4*a*c)) / (2*a)
                maneuverYComp = maneuverXComp * tan(radians(maneuverAngle))
                print(sqrt(maneuverXComp**2 + maneuverYComp**2), objAngles[x], objDistances[x])
                if(sqrt(maneuverXComp**2 + maneuverYComp**2) 
                        > maneuverDistance):
                    maneuverDistance = sqrt(maneuverXComp**2 + maneuverYComp**2)
                    closestObjAngle = objAngles[x]

    return maneuverDistance, closestObjAngle

def findTargetAngle(moveAngle, moveDistance, objAngle, objDistance):

    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################

    targetAngle = 0             ##The angle in which the machine will find the
                                ##object at closeAngle after repositioning with
                                ##moveAngle and moveDistance.
 
    targetDistance = 0.0        ##The corresponding distance value associated
                                ##with the targetAngle value.

    compMoveAngle = 0           ##The complement of the moveAngle value, that
                                ##is, the angle 180 degrees from moveAngle.

    vector1 = [0,0]             
    vector2 = [0,0]             
    vecTarg = [0,0]             ##The x and y components for compMoveAngle,
                                ##objAngle, and targetAngle.

    targAngCos = 0              ##The cosine of targetAngle that is used to
                                ##calculate the targetAngle value.

    #####################################

    if(moveAngle < 180):
        compMoveAngle = moveAngle + 180
    else:
        compMoveAngle = moveAngle - 180

    vector1[0] = cos(radians(compMoveAngle)) * moveDistance
    vector1[1] = sin(radians(compMoveAngle)) * moveDistance
    vector2[0] = cos(radians(objAngle)) * objDistance
    vector2[1] = sin(radians(objAngle)) * objDistance

    vecTarg[0] = vector1[0] + vector2[0]
    vecTarg[1] = vector1[1] + vector2[1]

    targetDistance = sqrt(vecTarg[0]**2 + vecTarg[1]**2)
    targAngCos = round(vecTarg[0]/targetDistance, 3)

    if(vecTarg[1] >= 0):
        targetAngle = degrees(acos(targAngCos))
    else:
        targetAngle = 360 - degrees(acos(targAngCos))

    print("\nMove   - D: %.3f, A: %3d\nObject - D: %.3f, A: %3d\nTarget - D: %.3f, A: %7.3f" % (moveDistance, compMoveAngle, objDistance, objAngle, targetDistance, targetAngle))

    return

def main():
    import random
    a = []
    d = []

    for _ in range(11):
        a.append(random.randint(0, 359))
        d.append(random.uniform(18.0, 25.0))

    ang = random.randint(0, 359)
    ret1 = findMoveDistance(a, d, ang, 42)

    findTargetAngle(ang, ret1[0], ret1[1], d[a.index(ret1[1])])

main()