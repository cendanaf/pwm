# For RaspberryPi GPIO

import RPi.GPIO as GPIO
from time import sleep

pwm0 = 12 #18
pwm1 = 13 #19

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pwm0, GPIO.OUT)
pwmPi = GPIO.PWM(pwm0, 1000) # Creates a PWM instance with 1kHz frequency
pwmPi.start(0)

for i in range(100):
    for duty in range(0, 101, 1):
        pwmPi.ChangeDutyCycle(duty)
        sleep(0.01)
    sleep(0.5)
    
    for duty in range(100, -1, -1):
        pwmPi.ChangeDutyCycle(duty)
        sleep(0.01)
    sleep(0.5)
