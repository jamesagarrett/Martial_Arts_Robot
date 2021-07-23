import RPi.GPIO as GPIO
import time
import numpy as np
import PCA9685

pwm = PCA9685.PCA9685()

hallPin = 10

GPIO.setmode(GPIO.BCM)
GPIO.setup(hallPin, GPIO.IN)
GPIO.setwarnings(False)

pwm.set_pwm_freq(160)
maxPWM = 15
inc = 5
CCW_MIN = 1085 #25
CW_MIN = 905 #65
PWM_PORTS = [5, 7, 6]
wheelRPMs = [[[0.0 for _ in range(int(maxPWM/inc)+1)] for _ in range(2)] for _ in range(3)]

def getRPM():
    revolvs = 10
    average = 0
    for _ in range (revolvs):
        while(GPIO.input(hallPin) == True):
            continue
        while(GPIO.input(hallPin) == False):
            continue

        start = time.time()
        while(GPIO.input(hallPin) == True):
            continue
        end = time.time()

        revolution = (1/(end-start))*60
        average += revolution

    average /= revolvs
    print(average)
    return average

funcCCW = [1085]
funcCW = [905]

try:
    for x in range (1):
    
        input('{}{}'.format("CCW Wheel ", x+1))
        for z in range(inc, maxPWM + inc, inc):
            pwm.set_pwm(PWM_PORTS[x], 0, CCW_MIN + z)
            time.sleep(0.1)
            wheelRPMs[x][0][int(z/inc)] = getRPM()
            funcCCW.append(CCW_MIN + z)

        pwm.set_pwm(PWM_PORTS[x], 0, 995)

        input('{}{}'.format("CW Wheel ", x+1))
        for z in range(inc , maxPWM + inc, inc):
            pwm.set_pwm(PWM_PORTS[x], 0, CW_MIN - z)
            time.sleep(0.1)
            wheelRPMs[x][1][int(z/inc)] = getRPM()
            funcCW.append(CW_MIN - z)

        pwm.set_pwm(PWM_PORTS[x], 0, 995)

    for x in range (1):
        for y in range (2):
            print(wheelRPMs[x][y])
	
    c = np.polyfit(wheelRPMs[0][0], funcCCW, 2)
    print(funcCCW, "\n")
    print("\nP = %.3f * R^2 + %.3f * R + %.3f\n" % (c[0], c[1], c[2]))
    c = np.polyfit(wheelRPMs[0][1], funcCW, 2)
    print(funcCW, "\n")
    print("\nP = %.3f * R^2 + %.3f * R + %.3f\n" % (c[0], c[1], c[2]))

    GPIO.cleanup()
except KeyboardInterrupt:
    pwm.set_pwm(PWM_PORTS[0], 0, 995)
    pwm.set_pwm(PWM_PORTS[1], 0, 995)
    pwm.set_pwm(PWM_PORTS[2], 0, 995)
    GPIO.cleanup()

