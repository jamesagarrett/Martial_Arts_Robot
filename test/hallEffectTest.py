import RPI.GPO as GPIO
import time

hallPin = 4

GPIO.setmode(GPIO_BOARD)
GPIO.setup(hallPin, GPIO.IN)
GPIO.setwarnings(False)

try:
    while True:
        if(GPIO.input(hallpin) == False):
            print("Magnet Detected!\n")
            time.sleep(0.5)
except KeyboardInterrupt:
    GPIO.cleanup()