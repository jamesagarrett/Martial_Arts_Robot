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
PWM_PORT = 5
wheelRPMsCCW = [0.0] * int(maxPWM/inc) + 1
wheelRPMsCW = [0.0] * int(maxPWM/inc) + 1
wheelPWMsCCW = [1085]
wheelPWMsCW = [905]

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

try:    
    input('{}{}'.format("CCW Wheel ", PWM_PORT))
    for z in range(inc, maxPWM + inc, inc):
        pwm.set_pwm(PWM_PORT, 0, CCW_MIN + z)
        time.sleep(0.1)
        wheelRPMsCCW[int(z/inc)] = getRPM()
        wheelPWMsCCW.append(CCW_MIN + z)

    for z in range(CCW_MIN + maxPWM, CCW_MIN - inc, -1 * inc):
        pwm.set_pwm(PWM_PORT, 0, z)
        time.sleep(0.1)
    pwm.set_pwm(PWM_PORT, 0, 995)

    input('{}{}'.format("CW Wheel ", PWM_PORT))
    for z in range(inc , maxPWM + inc, inc):
        pwm.set_pwm(PWM_PORT, 0, CW_MIN - z)
        time.sleep(0.1)
        wheelRPMsCW[int(z/inc)] = getRPM()
        wheelPWMsCW.append(CW_MIN - z)

    for z in range(CW_MIN - maxPWM, CW_MIN + inc, inc):
        pwm.set_pwm(PWM_PORT, 0, z)
        time.sleep(0.1)
    pwm.set_pwm(PWM_PORT, 0, 995)

    c = np.polyfit(wheelRPMsCCW, wheelPWMsCCW, 2)
    print("\n", wheelRPMsCCW, "\n", wheelPWMsCCW, "\n")
    print("\nP = %.3f * R^2 + %.3f * R + %.3f\n" % (c[0], c[1], c[2]))
    c = np.polyfit(wheelRPMsCW, wheelPWMsCW, 2)
    print("\n", wheelRPMsCW, "\n", wheelPWMsCW, "\n")
    print("\nP = %.3f * R^2 + %.3f * R + %.3f\n" % (c[0], c[1], c[2]))

    GPIO.cleanup()
except KeyboardInterrupt:
    pwm.set_pwm(PWM_PORT, 0, 995)
    GPIO.cleanup()

