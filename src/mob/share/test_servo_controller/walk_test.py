import serial
from time import sleep

try:
    sp = serial.Serial('/dev/ttyUSB0', 9600)
    print("Connected to "+str(sp)) 
    while True:
        print('Initial Position')
        sp.write('#5 P1600 #4 P1600 #3 P1600 #2 P1600 #1 P1500 #6 P1600 #7 P1600 #8 P1600 #9 P1600 #10 P1500 T500 \r')
        sleep(1)
        # stage 1
        sp.write('#5 P1600 #4 P1600 #3 P1600 #2 P1600 #1 P1500 #6 P1600 #7 P1600 #8 P1600 #9 P1600 #10 P1500 T500 \r')
        sleep(1)
        # stage 2
        sp.write('#5 P1600 #4 P1600 #3 P1600 #2 P1600 #1 P1500 #6 P1600 #7 P1600 #8 P1600 #9 P1600 #10 P1500 T500 \r')
        sleep(1)
        # stage 3
        sp.write('#5 P1600 #4 P1600 #3 P1600 #2 P1600 #1 P1500 #6 P1600 #7 P1600 #8 P1600 #9 P1600 #10 P1500 T500 \r')
        sleep(1)
        # stage 4
        sp.write('#5 P1600 #4 P1600 #3 P1600 #2 P1600 #1 P1500 #6 P1600 #7 P1600 #8 P1600 #9 P1600 #10 P1500 T500 \r')
        sleep(1)
        # stage 5
        sp.write('#5 P1600 #4 P1600 #3 P1600 #2 P1600 #1 P1500 #6 P1600 #7 P1600 #8 P1600 #9 P1600 #10 P1500 T500 \r')
        sleep(1)
except KeyboardInterrupt:
    print("CTRL-C press closing program")
    sp.write('#5 P1600 #4 P1600 #3 P1600 #2 P1600 #1 P1500 #6 P1600 #7 P1600 #8 P1600 #9 P1600 #10 P1500 T500 \r')
    sp.close()
