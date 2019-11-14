import serial
from time import sleep

try:
    sp = serial.Serial('/dev/ttyUSB0', 9600)
    print("Connected to "+str(sp)) 
    while True:
        print('start')
        sp.write('#5 P1500 T500 \r')
        sleep(1)
        sp.write('#5 P2500 T750 \r')
        sleep(1)
        sp.write('#5 P1500 T750 \r')
        sleep(1)

except Exception as e:
    print("FAILURE:: ",e)
    sp.close()
