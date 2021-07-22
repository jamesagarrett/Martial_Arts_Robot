import RPi.GPIO as GPIO
import time
import PCA9685

pwm = PCA9685.PCA9685()

hallPin = 7

GPIO.setmode(GPIO.BOARD)
GPIO.setup(hallPin, GPIO.IN)
GPIO.setwarnings(False)

pwm.set_pwm_freq(160)
maxPWM = 15
CCW_MIN = 1045#25
CW_MIN = 945#65
PWM_PORTS = [7, 5, 6]
wheelRPMs = [[[0 for _ in range(maxPWM)] for _ in range(2)] for _ in range(3)]

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
    return average

try:
    for x in range (3):
    
        input("CCW Wheel ", x+1)
        for z in range(maxPWM):
            pwm.set_pwm(PWM_PORTS[x], 0, CCW_MIN + z + 1)
            time.sleep(0.1)
            wheelRPMs[x][0][z] = getRPM()

        input("CW Wheel ", x+1)
        for z in range(maxPWM):
            pwm.set_pwm(PWM_PORTS[x], 0, CW_MIN - z - 1)
            time.sleep(0.1)
            wheelRPMs[x][1][z] = getRPM()

        pwm.set_pwm(PWM_PORTS[x], 0, 995)

    for x in range (3):
        for y in range (2):
            print(wheelRPMs[x][y])

    GPIO.cleanup()
except:
    pwm.set_pwm(PWM_PORTS[0], 0, 995)
    pwm.set_pwm(PWM_PORTS[1], 0, 995)
    pwm.set_pwm(PWM_PORTS[2], 0, 995)
    GPIO.cleanup()

