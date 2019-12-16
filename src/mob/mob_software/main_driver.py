#!/usr/bin/python3

# custom modules
import mob_config as mob
import gyroscope
import sonar
# from pid import PID
from PIDController_old import pid

# dependancy
import serial
import time
from threading import Thread


class main:
    def __init__(self):
        # head
        self.cam = mob.cam
        self.head = mob.head

        # right arm
        self.right_shoulder = mob.right_shoulder
        self.right_elbow = mob.right_elbow
        self.right_hand = mob.right_hand

        # left arm
        self.left_shoulder = mob.left_shoulder
        self.left_elbow = mob.left_elbow
        self.left_hand = mob.left_hand

        # right leg
        self.right_hip = mob.right_hip
        self.right_thigh = mob.right_thigh
        self.right_knee = mob.right_knee
        self.right_ankle = mob.right_ankle
        self.right_foot = mob.right_foot

        # left leg
        self.left_hip = mob.left_hip
        self.left_thigh = mob.left_thigh
        self.left_knee = mob.left_knee
        self.left_ankle = mob.left_ankle
        self.left_foot = mob.left_foot

        # sensors
        # gyro_thread  = Thread(target=gyroscope.gyroscope())
        # sonar_thread = Thread(target=sonar.sonar(), args=args, kwargs=kwargs)

        # controller
        self.pid = pid(left_hip, right_hip, gyroscope)

        self.driver()

    def driver(self):
        # establish connection
        connection = serial.Serial('/dev/ttyUSB0', 9600)
        # initial position
        connection.write('#14 P1500 #15 P1500 #16 P1500 #17 P1600 #18 P1500 #19 P1700 #20 P1600 #21 P1600 #22 P1500 '
                         '#23 P1600 #24 P1600 #25 P1750 #26 P1500 #27 P1640 #28 P1500 #29 P1600 #30 P1500 #31 P1500 '
                         'T1000 \r')

        while connection:
            Ax,Ay,Az,Gx,Gz,Gy = gyroscope.gyroscope
            pid          = self.pid
            print("sonar cm", sonar_cm)
            print("sonar inch", sonar_inch)
            print("Gyro: "+str(Ax)+" "+str(Ay)+str(Az))
            time.sleep(2)


if __name__ == '__main__':
    main()
