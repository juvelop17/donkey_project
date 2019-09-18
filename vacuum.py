import RPi.GPIO as GPIO
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

        GPIO.output(self.pin1, False)
        GPIO.output(self.pin2, True)

    def run(self):
        GPIO.output(self.pin3, True)
        print('청소기 작동')

    def cancel(self):
        GPIO.output(self.pin3, False)
        print('청소기 정지')


if __name__ == '__main__':
	vacuumMortor = VacuumMortor()
	vacuumMortor.run() 
	time.sleep(5)
	vacuumMortor.cancel()