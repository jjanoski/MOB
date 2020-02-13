#! /usr/bin/python python
import serial
from time import sleep


def angle_command(ser, joint, angle):
    set_value = int(angle * 11.11 + 501)
    print('Angle to Microseconds: '+str(set_value))
    command = '#'+str(joint)+' P'+str(set_value)+' T1000 \r'
    print('Command: '+str(command))
    ser.write(command)
    ser.flush()

sp = serial.Serial('/dev/ttyUSB0', 9600)
print("Connection successful")
 
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

while True:
    print('*******************************************')
    print('*            Testing TOOl                 *')
    print('*                                         *')
    print('* Motors goes from 0 - 31                 *')
    print('* Head:      14      || VR CAM:   15      *')
    print('* Right Leg: 16 - 20 || Left Leg: 21 - 25 *')
    print('* Right Arm: 26 - 28 || Left Arm: 29 - 31 *')
    print('* Enter 32 to shutdown                    *')
    print('*                                         *')
    print('* Rates Initial 1500, Min 500, Max 2500   *')
    print('*******************************************\n')
    
    try:
        # get motor from user
        joint = input('Input motor choice: ')

        if joint == 32:
            print('close') 
            print('setting legs to initial')
            sp.write('#16 P1500 #17 P1600 #18 P1500 #19 P1700 #20 P1600 #21 P1600 #22 P1500 #23 P1600 #24 P1600 #25 P1750 T1000 \r')
            sleep(2)
            print('setting head to initial')
            sp.write('#14 P1500 #15 P1500 T500 \r')
            sleep(2)
            print('setting arms to inital')
            sp.write('#26 P1500 #27 P1700 #28 P1500 #29 P1600 #30 P1500 #31 P1500 T500 \r')
            sleep(2)
            print('Finished')
            break
            sp.close()

        # get angle rate from user
        angle = input('Input Angle: ')
        if angle > -1 and angle < 181:
            # Send motor signal to execute on robot
            print("Command Executing")
            angle_command(sp,joint, angle)
            sleep(2)
        else:
            print("Rate out of bounds, try again")
            sleep(2) 
    
        print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
    except Exception as e:
        print("Error: "+str(e)+"\n")

sp.close()
