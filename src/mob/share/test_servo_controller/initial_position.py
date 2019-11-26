import serial
from time import sleep

sp = serial.Serial('/dev/ttyUSB0', 9600)
print("Connected to "+str(sp))
 
"""
key map
---------------------
right     | left
---------------------
16: hip   | 21:  hip
17: thigh | 22:  thigh
18: shin  | 23:  shin
19: ankle | 24:  ankle
20: foot  | 25:  foot
"""
        
print('Initial Position')
#stage 1
sp.write('#16 P1500 #17 P1600 #18 P1500 #19 P1700 #20 P1600 #21 P1600 #22 P1500 #23 P1600 #24 P1600 #25 P1750 T1000 \r')
sleep(2)

sp.close()
