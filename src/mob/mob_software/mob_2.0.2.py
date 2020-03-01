#!/usr/bin/python3.7
import smbus
import serial
import sys
import math
import matplotlib.pyplot as plt
from time import sleep, time

# start here tomorrow
class MobSoft:
    def __init__(self): 
        # Run main function
        sp = serial.Serial('/dev/ttyUSB0', 9600)
        self.run(sp)

    def angle_command_multiple(self,ser,
                    angle_left_shoulder,
                    angle_left_elbow,
                    angle_left_wrist,  
                    angle_left_hand,
                    angle_left_thigh,
                    angle_left_knee,
                    angle_left_foot,
                    angle_right_shoulder,
                    angle_right_elbow,
                    angle_right_wrist,
                    angle_right_hand,
                    angle_right_thigh,
                    angle_right_knee,
                    angle_right_foot,
                    ):
        """ Send angle command to the robot motors"""
        set_l_shoulder = int(angle_left_shoulder * 11.11 + 501)
        set_l_elbow = int(angle_left_elbow * 11.11 + 501)
        set_l_wrist = int(angle_left_wrist * 11.11 + 501)
        set_l_hand = int(angle_left_hand * 11.11 + 501)
        set_l_thigh = int(angle_left_thigh * 11.11 + 501)
        set_l_knee = int(angle_left_knee * 11.11 + 501)
        set_l_foot = int(angle_left_foot * 11.11 + 501)
        set_r_shoulder = int(angle_right_shoulder * 11.11 + 501)
        set_r_elbow = int(angle_right_elbow * 11.11 + 501)
        set_r_wrist = int(angle_right_wrist * 11.11 + 501)
        set_r_hand = int(angle_right_hand * 11.11 + 501)
        set_r_thigh = int(angle_right_thigh * 11.11 + 501)
        set_r_knee = int(angle_right_knee * 11.11 + 501)
        set_r_foot = int(angle_right_foot * 11.11 + 501)
        left_shoulder  = '0' 
        left_elbow     = '1'
        left_wrist     = '2'
        left_hand      = '3'
        
        left_thigh     = '24'
        left_knee      = '25'
        left_foot      = '11'
        
        right_shoulder = '4'
        right_elbow    = '5'
        right_wrist    = '6'
        right_hand     = '7'
        
        right_thigh    = '31' 
        right_knee     = '29' 
        right_foot     = '30' 
        
        #print(str(set_value))
        if 499 < set_l_shoulder < 2501 and 499 < set_l_elbow < 2501 and 499 < set_l_wrist < 2501 and 499 < set_l_hand < 2501 and 499 < set_l_thigh < 2501 and 499 < set_l_knee < 2501 and 499 < set_l_thigh < 2501 and 499 < set_l_knee < 2501 and 499 < set_l_foot < 2501 and 499 < set_r_shoulder < 2501 and 499 < set_r_elbow < 2501 and 499 < set_r_wrist < 2501 and 499 < set_r_hand < 2501 and 499 < set_r_thigh < 2501 and 499 < set_r_knee < 2501 and 499 < set_r_foot < 2501 :
            command = '#'+left_shoulder+' P'+str(set_l_shoulder)+' #'+left_elbow+' P'+str(set_l_elbow)+' #'+left_wrist+' P'+str(set_l_wrist)+' #'+left_hand+' P'+str(set_l_hand)+' #'+left_thigh+' P'+str(set_l_thigh)+' #'+left_knee+' P'+str(set_l_knee)+' #'+left_foot+' P'+str(set_l_foot)+' #'+right_shoulder+' P'+str(set_r_shoulder)+' #'+right_elbow+' P'+str(set_r_elbow)+' #'+right_wrist+' P'+str(set_r_wrist)+' #'+right_hand+' P'+str(set_r_hand)+' #'+right_thigh+' P'+str(set_r_thigh)+' #'+right_knee+' P'+str(set_r_knee)+' #'+right_foot+' P'+str(set_r_foot)+' T1000 \r'
            print(command)
            ser.write(command.encode())
            #ser.flush()
        else:
            print("Out of Bounds")
    
    def stand(self, sp):
        print("standing")
        # Left Side
        left_shoulder_out = 90
        left_elbow_out    = 90
        left_wrist_out    = 90
        left_hand_out     = 5
        
        left_thigh_out    = 70
        left_knee_out     = 50
        left_foot_out     = 90
        
        # Right side
        right_shoulder_out = 90
        right_elbow_out    = 90
        right_wrist_out    = 90
        right_hand_out     = 170
        
        right_thigh_out    = 95
        right_knee_out     = 90
        right_foot_out     = 100
        self.angle_command_multiple(sp, left_shoulder_out, left_elbow_out, left_wrist_out, left_hand_out, left_thigh_out, left_knee_out, left_foot_out, right_shoulder_out, right_elbow_out, right_wrist_out, right_hand_out, right_thigh_out, right_knee_out, right_foot_out,)
    
    def walk_1_1(self, sp):        
        print("step 1.1")
        # Left Side
        left_shoulder_out = 90
        left_elbow_out    = 90
        left_wrist_out    = 90
        left_hand_out     = 5
        
        left_thigh_out    = 70
        left_knee_out     = 50
        left_foot_out     = 80
        
        # Right side
        right_shoulder_out = 90
        right_elbow_out    = 90
        right_wrist_out    = 90
        right_hand_out     = 170
        
        right_thigh_out    = 95
        right_knee_out     = 90
        right_foot_out     = 90
        self.angle_command_multiple(sp, left_shoulder_out, left_elbow_out, left_wrist_out, left_hand_out, left_thigh_out, left_knee_out, left_foot_out, right_shoulder_out, right_elbow_out, right_wrist_out, right_hand_out, right_thigh_out, right_knee_out, right_foot_out,)
     
    def walk_1_2(self, sp):        
        print("step 1.2")
        # Left Side
        left_shoulder_out = 90
        left_elbow_out    = 90
        left_wrist_out    = 90
        left_hand_out     = 5
        
        left_thigh_out    = 70
        left_knee_out     = 50
        left_foot_out     = 80
        
        # Right side
        right_shoulder_out = 90
        right_elbow_out    = 90
        right_wrist_out    = 90
        right_hand_out     = 170
        
        right_thigh_out    = 120
        right_knee_out     = 120
        right_foot_out     = 90
        self.angle_command_multiple(sp, left_shoulder_out, left_elbow_out, left_wrist_out, left_hand_out, left_thigh_out, left_knee_out, left_foot_out, right_shoulder_out, right_elbow_out, right_wrist_out, right_hand_out, right_thigh_out, right_knee_out, right_foot_out,)

    def walk_1_3(self, sp):        
        print("step 1.3")
        # Left Side
        left_shoulder_out = 90
        left_elbow_out    = 90
        left_wrist_out    = 90
        left_hand_out     = 5
        
        left_thigh_out    = 70
        left_knee_out     = 50
        left_foot_out     = 80
        
        # Right side
        right_shoulder_out = 90
        right_elbow_out    = 90
        right_wrist_out    = 90
        right_hand_out     = 170
        
        right_thigh_out    = 120
        right_knee_out     = 120
        right_foot_out     = 100
        self.angle_command_multiple(sp, left_shoulder_out, left_elbow_out, left_wrist_out, left_hand_out, left_thigh_out, left_knee_out, left_foot_out, right_shoulder_out, right_elbow_out, right_wrist_out, right_hand_out, right_thigh_out, right_knee_out, right_foot_out,)
     
    def walk_2_1(self, sp):
        print("step 2.1")
        
        # Left Side
        left_shoulder_out = 90
        left_elbow_out    = 90
        left_wrist_out    = 90
        left_hand_out     = 5
        
        left_thigh_out    = 80
        left_knee_out     = 50
        left_foot_out     = 115
        
        # Right side
        right_shoulder_out = 90
        right_elbow_out    = 90
        right_wrist_out    = 90
        right_hand_out     = 170
        
        right_thigh_out    = 95
        right_knee_out     = 90
        right_foot_out     = 114
        self.angle_command_multiple(sp, left_shoulder_out, left_elbow_out, left_wrist_out, left_hand_out, left_thigh_out, left_knee_out, left_foot_out, right_shoulder_out, right_elbow_out, right_wrist_out, right_hand_out, right_thigh_out, right_knee_out, right_foot_out,)
    
    def walk_2_2(self, sp):
        print("step 2.2")
        
        # Left Side
        left_shoulder_out = 90
        left_elbow_out    = 90
        left_wrist_out    = 90
        left_hand_out     = 5
        
        left_thigh_out    = 40
        left_knee_out     = 6
        left_foot_out     = 90
        
        # Right side
        right_shoulder_out = 90
        right_elbow_out    = 90
        right_wrist_out    = 90
        right_hand_out     = 170
        
        right_thigh_out    = 95
        right_knee_out     = 90
        right_foot_out     = 114
        self.angle_command_multiple(sp, left_shoulder_out, left_elbow_out, left_wrist_out, left_hand_out, left_thigh_out, left_knee_out, left_foot_out, right_shoulder_out, right_elbow_out, right_wrist_out, right_hand_out, right_thigh_out, right_knee_out, right_foot_out,)
    
    def walk_2_3(self, sp):
        print("step 2.3")
        
        # Left Side
        left_shoulder_out = 90
        left_elbow_out    = 90
        left_wrist_out    = 90
        left_hand_out     = 5
        
        left_thigh_out    = 40
        left_knee_out     = 30
        left_foot_out     = 85
        
        # Right side
        right_shoulder_out = 90
        right_elbow_out    = 90
        right_wrist_out    = 90
        right_hand_out     = 170
        
        right_thigh_out    = 95
        right_knee_out     = 90
        right_foot_out     = 110
        self.angle_command_multiple(sp, left_shoulder_out, left_elbow_out, left_wrist_out, left_hand_out, left_thigh_out, left_knee_out, left_foot_out, right_shoulder_out, right_elbow_out, right_wrist_out, right_hand_out, right_thigh_out, right_knee_out, right_foot_out,)
    
    def walk(self,sp):
        # LEFT FOOT Stages -- stage 3 needs tuning
        self.walk_1_1(sp)
        sleep(1)
        self.walk_1_2(sp)
        sleep(1)
        self.walk_1_3(sp)
        sleep(1)
        self.stand(sp)
        sleep(2)
        # RIGHT FOOT Stages --- Done
        self.walk_2_1(sp)
        sleep(1)
        self.walk_2_2(sp)
        sleep(1)
        self.walk_2_3(sp)
        sleep(1)
        self.stand(sp)
        sleep(15)     
    
    def run(self, sp):
        i = 0        
        while True:
            try:
                self.stand(sp)
                sleep(2)
                self.walk(sp)
                #user_command = input('Movement Command: ')
                #user_command = str(user_command).lower()
                #print("command: "+user_command)
                #self.stand(sp)
                #if 'w' in user_command:
                #    self.walk(sp)
                #if 'q' in user_command:
                #    sys.exit()
                i = i +1
                print(i)
                      
            except Exception as e:
                print("ERROR, ", e)
                

if __name__ == '__main__':
    MobSoft()



