import serial
from time import sleep

try:
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

    print('FULL F**KING SEND!!!!!!!!!!!!')
    #stage 1
    sp.write('#10 P500 #16 P1500 #17 P2500 #18 P2500 #19 P1500 #20 P1600 #21 P1600 #22 P1500 #23 P1700 #24 P1500 #25 P1600 T1000 \r')

except KeyboardInterrupt:
    print("CTRL-C press closing program")
    sp.write('#16 P1500 #17 P1600 #18 P1700 #19 P1500 #20 P1600 #21 P1600 #22 P1500 #23 P1600 #24 P1800 #25 P1500 T2000 \r')
    sp.write('#16 P1500 #17 P1600 #18 P1700 #19 P1500 #20 P1600 #21 P1600 #22 P1500 #23 P1700 #24 P1700 #25 P1500 T2000 \r')
    sp.close()
