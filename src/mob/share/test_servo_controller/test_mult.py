import serial
from time import sleep

try:
    sp = serial.Serial('/dev/ttyUSB0', 9600)
    print("Connected to "+str(sp)) 
    while True:
        print('initial position')
	#sp.write('#5 P1600 #4 P1600 #3 P1500 #2 P1600 #1 P1500 #6 P1600 #7 P1600 #8 P1400 #9 P1400 #10 P1500 T500 \r')
        print('start')
        sp.write('#5 P1250 T500 \r')
        sp.write('#10 P1750 T500 \r')
        sleep(1)
        sp.write('#4 P1250 T500 \r')
        sp.write('#9 P1750 T500 \r')
        sleep(1)
	sp.write('#3 P1250 T500 \r')
        sp.write('#8 P1750 T500 \r')
        sleep(1)
	sp.write('#2 P1250 T500 \r')
        sp.write('#7 P1750 T500 \r')
        sleep(1)
	sp.write('#1 P1250 T500 \r')
        sp.write('#6 P1750 T500 \r')
        sleep(1)
        print('finish')
        sp.write('#5 P1600 #4 P1600 #3 P1600 #2 P1600 #1 P1500 #6 P1600 #7 P1600 #8 P1600 #9 P1600 #10 P1500 T500 \r')
        sleep(1)
except KeyboardInterrupt:
    print("CTRL-C press closing program")
    sp.write('#5 P1600 #4 P1600 #3 P1600 #2 P1600 #1 P1500 #6 P1600 #7 P1600 #8 P1600 #9 P1600 #10 P1500 T500 \r')
    sp.close()
