import serial
from time import sleep

try:
    sp = serial.Serial('/dev/ttyUSB0', 9600)
    print("Connected to "+str(sp)) 
    print('initial position')
    sp.write('#5 P1600 #4 P1600 #3 P1600 #2 P1600 #1 P1500 #6 P1600 #7 P1600 #8 P1600 #9 P1600 #10 P1500 T500 \r') # initial position
    sp.close()
except Exception as e:
    print("FAILURE:: ",e)
    sp.close()
