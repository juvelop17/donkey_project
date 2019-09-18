import RPi.GPIO as GPIO
import sys
import time




class VacuumMortor:
    pin1 = 17  # 방향 1
    pin2 = 27  # 방향 2
    pin3 = 22  # PWM 신호

    def __init__(self):
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.pin1, GPIO.OUT)
        GPIO.setup(self.pin2, GPIO.OUT)
        GPIO.setup(self.pin3, GPIO.OUT)

    def run(self):
        GPIO.output(self.pin1, True)
        GPIO.output(self.pin2, False)
        GPIO.output(self.pin3, True)

        print('Motor 1')
        time.sleep(0.1)

