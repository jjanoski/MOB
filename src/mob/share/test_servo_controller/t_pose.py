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
        
print('wave 1')
#stage 1
sp.write('#26 P1500 #27 P1640 #28 P1500 T1000 \r')
sp.write('#29 P1600 #30 P1600 #31 P1500 T1000 \r')
sleep(2)

print('wave 2')
#stage 2
sp.write('#27 P2500 #30 P700 T1000 \r')
sleep(3)

print('wave 3')
#stage 3
sp.write('#26 P1500 #27 P1600 #28 P1500 T1000 \r')
sp.write('#29 P1600 #30 P1600 #31 P1500 T1000 \r')
sleep(2)

sp.close()
