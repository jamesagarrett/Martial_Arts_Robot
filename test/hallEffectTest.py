import RPi.GPIO as GPIO
import time
import numpy as np
from matplotlib import pyplot as plt
import PCA9685

pwm = PCA9685.PCA9685()

hallPin = 21 #7 21 5 04 6 10

GPIO.setmode(GPIO.BCM)
GPIO.setup(hallPin, GPIO.IN)
GPIO.setwarnings(False)

pwm.set_pwm_freq(160)
PWM_PORT = 7
maxPWM = 460 
step = 5
CCW_MIN = 1060#7 1060 5 1030 6 1040 
CW_MIN = 940#7 940 5 965 6 950 
wheelRPMsCCW = []
wheelRPMsCW = []
wheelPWMsCCW = []
wheelPWMsCW = []

def getRPM(pwmChange):
    revolvs = 10
    average = 0

    for _ in range (revolvs):
        while(GPIO.input(hallPin) == True):
            continue

        while(GPIO.input(hallPin) == False):
            start = time.time()
        while(GPIO.input(hallPin) == True):
            end = time.time()

        revolution = 60/(end-start)
        average += revolution

    average /= revolvs
    print(pwmChange, average)
    return average

try:    
    input('{}{}'.format("CCW Wheel ", PWM_PORT))
    for z in range(step, maxPWM + step, step):
        pwm.set_pwm(PWM_PORT, 0, CCW_MIN + z)
        time.sleep(0.1)
        wheelRPMsCCW.append(getRPM(z))
        wheelPWMsCCW.append(CCW_MIN + z)

    for z in range(CCW_MIN + maxPWM, CCW_MIN - step, -1 * step):
        pwm.set_pwm(PWM_PORT, 0, z)
        time.sleep(0.1)
    pwm.set_pwm(PWM_PORT, 0, 995)

    input('{}{}'.format("CW Wheel ", PWM_PORT))
    for z in range(step , maxPWM + step, step):
        pwm.set_pwm(PWM_PORT, 0, CW_MIN - z)
        time.sleep(0.1)
        wheelRPMsCW.append(getRPM(z))
        wheelPWMsCW.append(CW_MIN - z)

    for z in range(CW_MIN - maxPWM, CW_MIN + step, step):
        pwm.set_pwm(PWM_PORT, 0, z)
        time.sleep(0.1)
    pwm.set_pwm(PWM_PORT, 0, 995)

    xMin = 0
    xMax = max(max(wheelRPMsCCW), max(wheelRPMsCW)) + 50
    yMin = CW_MIN - maxPWM - 20
    yMax = CCW_MIN + maxPWM + 20

    c = np.polyfit(wheelRPMsCCW, wheelPWMsCCW, 2)
    y = [c[0]*x**2 + c[1]*x + c[2] for x in wheelRPMsCCW]
    for x in range(1, int(maxPWM/step + 1), 5):
        print("\n", wheelRPMsCCW[x:x+5], "\n", wheelPWMsCCW[x:x+5], "\n")
    eq = "CCW: P = %8.5f R^2 + %8.5f R + %10.5f" % (c[0], c[1], c[2])
    plot_CCW = plt.scatter(wheelRPMsCCW, wheelPWMsCCW, color='orange')
    plt.plot(wheelRPMsCCW, y, color='black')
    plt.text(xMin + 25, yMin + 75, eq, fontsize=6.5)

    for _ in range(2):
        print("\t===================================")

    c = np.polyfit(wheelRPMsCW, wheelPWMsCW, 2)
    y = [c[0]*x**2 + c[1]*x + c[2] for x in wheelRPMsCW]
    for x in range(1, int(maxPWM/step + 1), 5):
        print("\n", wheelRPMsCW[x:x+5], "\n", wheelPWMsCW[x:x+5], "\n")
    eq = "CW:   P = %8.5f R^2 + %8.5f R + %10.5f" % (c[0], c[1], c[2])
    plot_CW = plt.scatter(wheelRPMsCW, wheelPWMsCW, color='violet')
    plt.plot(wheelRPMsCW, y, color='black')
    plt.text(xMin + 25, yMin + 25, eq, fontsize=6.5)

    plt.title('{}{}{}'.format("Wheel ", PWM_PORT, " Graph"))
    plt.xlabel("RPM")
    plt.ylabel("PWM")
    plt.xlim([0, xMax])
    plt.ylim([yMin, yMax])
    plt.legend([plot_CCW, plot_CW], ["Counter-Clockwise","Clockwise"])
    plt.show()

    GPIO.cleanup()
except KeyboardInterrupt:
    pwm.set_pwm(PWM_PORT, 0, 995)
    GPIO.cleanup()
    print("\n")

