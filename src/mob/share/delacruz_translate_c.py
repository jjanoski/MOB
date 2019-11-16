import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)

# clean RPi pins
GPIO.output(11, 0)

time.sleep(0.000002)

# send signal
GPIO.output(11, 1)

time.sleep(0.000005)

GPIO.output(11,0)

GPIO.setup(11, GPIO.IN)

while GPIO.input(11)==0:
    starttime = time.time()

while GPIO.input(11)==1:
    endtime=time.time()

duration = endtime - starttime
distance = duration*34000/2

print("Distance: "+str(distance))

