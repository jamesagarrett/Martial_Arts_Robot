import RPi.GPIO as GPIO
import time

hallPin = 7

GPIO.setmode(GPIO.BOARD)
GPIO.setup(hallPin, GPIO.IN)
GPIO.setwarnings(False)

try:
    while True:
        if(GPIO.input(hallPin) == False):
            print("Magnet Detected!\n")
            time.sleep(0.25)
        else:
            print("No Magnet...\n")
            time.sleep(0.25)
except KeyboardInterrupt:
    GPIO.cleanup()
