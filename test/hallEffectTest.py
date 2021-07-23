import RPi.GPIO as GPIO
import time
import numpy as np
from matplotlib import pyplot as plt
import PCA9685

pwm = PCA9685.PCA9685()

hallPin = 10

GPIO.setmode(GPIO.BCM)
GPIO.setup(hallPin, GPIO.IN)
GPIO.setwarnings(False)

pwm.set_pwm_freq(160)
maxPWM = 500
step = 5
CCW_MIN = 1080 #25
CW_MIN = 910 #65
PWM_PORT = 5
wheelRPMsCCW = [0.0] * int(maxPWM/step + 1)
wheelRPMsCW = [0.0] * int(maxPWM/step + 1)
wheelPWMsCCW = [CCW_MIN]
wheelPWMsCW = [CW_MIN]

def getRPM():
    revolvs = 10
    average = 0

    for _ in range (revolvs):
        while(GPIO.input(hallPin) == True):
            continue

        while(GPIO.input(hallPin) == False):
            start = time.time()
        while(GPIO.input(hallPin) == True):
            end = time.time()

        revolution = (1/(end-start))*60
        average += revolution

    average /= revolvs
    print(average)
    return average

try:    
    input('{}{}'.format("CCW Wheel ", PWM_PORT))
    for z in range(step, maxPWM + step, step):
        pwm.set_pwm(PWM_PORT, 0, CCW_MIN + z)
        time.sleep(0.1)
        wheelRPMsCCW[int(z/step)] = getRPM()
        wheelPWMsCCW.append(CCW_MIN + z)

    for z in range(CCW_MIN + maxPWM, CCW_MIN - step, -1 * step):
        pwm.set_pwm(PWM_PORT, 0, z)
        time.sleep(0.05)
    pwm.set_pwm(PWM_PORT, 0, 995)

    input('{}{}'.format("CW Wheel ", PWM_PORT))
    for z in range(step , maxPWM + step, step):
        pwm.set_pwm(PWM_PORT, 0, CW_MIN - z)
        time.sleep(0.1)
        wheelRPMsCW[int(z/step)] = getRPM()
        wheelPWMsCW.append(CW_MIN - z)

    for z in range(CW_MIN - maxPWM, CW_MIN + step, step):
        pwm.set_pwm(PWM_PORT, 0, z)
        time.sleep(0.05)
    pwm.set_pwm(PWM_PORT, 0, 995)

    xMin = 0
    xMax = max(max(wheelRPMsCCW), max(wheelRPMsCW)) + 50
    yMin = CW_MIN - maxPWM - 20
    yMax = CCW_MIN + maxPWM + 20

    c = np.polyfit(wheelRPMsCCW, wheelPWMsCCW, 2)
    y = [c[0]*x**2 + c[1]*x + c[2] for x in wheelRPMsCCW]
    for x in range(1, int(maxPWM/step + 1), 5):
        print("\n", wheelRPMsCCW[x:x+5], "\n", wheelPWMsCCW[x:x+5], "\n")
    eq = "CCW: P = %7.4f R^2 + %7.4f R + %9.4f" % (c[0], c[1], c[2])
    plot_CCW = plt.scatter(wheelRPMsCCW, wheelPWMsCCW, color='orange')
    plt.plot(wheelRPMsCCW, y, color='black')
    plt.text(xMin + 25, yMin + 75, eq, fontsize=6.5)

    for _ in range(2):
        print("\t===================================")

    c = np.polyfit(wheelRPMsCW, wheelPWMsCW, 2)
    y = [c[0]*x**2 + c[1]*x + c[2] for x in wheelRPMsCW]
    for x in range(1, int(maxPWM/step + 1), 5):
        print("\n", wheelRPMsCW[x:x+5], "\n", wheelPWMsCW[x:x+5], "\n")
    eq = "CW:   P = %7.4f R^2 + %7.4f R + %9.4f" % (c[0], c[1], c[2])
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

