#!/usr/bin/python python

import time
import sys
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)

class sonar():
    def __init__(self):
        self.distance_cm   = None
        self.distance_inch = None
        self.driver()

    def driver(self):
        while True:
            self.distance_cm = self.read_distance(11)
            self.distance_inch = self.distance_cm/2.54
            print("distnace: "+str(round(self.distance_cm, 2))+" cm")
            print("distnace: "+str(round(self.distance_inch, 2))+" in")
            time.sleep(1)

    def read_distance(self, pin):
        # pin configuration
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, 0)
        time.sleep(0.000002)
        GPIO.output(pin, 1)
        time.sleep(0.000005)
        GPIO.output(pin, 0)
        GPIO.setup(pin, GPIO.IN)
        # get stat time
        while GPIO.input(pin)==0:
            start_time=time.time()
        # get end time
        while GPIO.input(pin)==1:
            end_time = time.time()

        # calculation duraiton and distance 
        duration = self.get_duration(start_time, end_time)
        distance = self.get_cm_distance(duration)
        # return disntace in cm
        return distance

    def get_duration(self, start, end):
        duration_time = end - start
        return duration_time

    def get_cm_distance(self, duration):
        distance_in_cm = duration * 34000/2
        return distance_in_cm
        

if __name__ == '__main__':
    sonar()
