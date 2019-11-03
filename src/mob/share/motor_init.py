#!/usr/bin/env python

# created by Jiovanni Janoski
# test motor directly

# obtain required libraries
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.OUT)

p = GPIO.PWM(7, 50)
p.start(7.5)

try:
    while True:
        print("Neutral")
        p.ChangeDutyCycle(7.5) # Neutral
        time.sleep(1)
        
except KeyboardInterrupt:
    p.ChangeDutyCycle(7.5)
    time.sleep(1)
    p.stop()
    GPIO.cleanup()
