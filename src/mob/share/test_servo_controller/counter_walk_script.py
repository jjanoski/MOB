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

    print('Stage 1 - Initial Position')
    #stage 1
    sp.write('#16 P1500 #17 P1600 #18 P1500 #19 P1600 #20 P1600 #21 P1600 #22 P1500 #23 P1600 #24 P1600 #25 P1550 T1000 \r')
    sleep(2)

    print('Stage 2 - Lean Left')
    # stage 2
    sp.write('#16 P1500 #17 P1600 #18 P1500 #19 P1600 #20 P1600 #21 P1800 #22 P1500 #23 P1600 #24 P1600 #25 P1550 T1000 \r')
    sleep(2)

    #print('Stage 3 - Lift Right Foot and Lean Left')
    # stage 3
    #sp.write('#16 P1400 #17 P2500 #18 P2500 #19 P1400 #20 P1600 #21 P1500 #22 P1650 #23 P1700 #24 P1500 #25 P1550 T2000 \r')
    #sleep(10)

    #print('Stage 4')
    # stage 4
    #sp.write('#16 P1500 #17 P1600 #18 P1700 #19 P1600 #20 P1404 #21 P1600 #22 P1600 #23 P1700 #24 P1600 #25 P1565 T500 \r')
    #sleep(2)
    #print('Stage 5')
    # stage 5
    #sp.write('#16 P1500 #17 P1974 #18 P1700 #19 P1752 #20 P1426 #21 P1600 #22 P1600 #23 P1700 #24 P1600 #25 P1739 T500 \r')
    #sleep(2)

    #sp.write('#16 P1500 #17 P1600 #18 P1500 #19 P1600 #20 P1500 #21 P1600 #22 P1500 #23 P1600 #24 P1600 #25 P1500 T1000 \r')
except KeyboardInterrupt:
    print("CTRL-C press closing program")
    sp.write('#16 P1500 #17 P1600 #18 P1700 #19 P1500 #20 P1600 #21 P1600 #22 P1500 #23 P1600 #24 P1800 #25 P1500 T2000 \r')
    sp.write('#16 P1500 #17 P1600 #18 P1700 #19 P1500 #20 P1600 #21 P1600 #22 P1500 #23 P1700 #24 P1700 #25 P1500 T2000 \r')
    sp.close()
