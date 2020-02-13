#!/usr/bin/python3.7
import smbus
import serial
import sys
import math
import matplotlib.pyplot as plt
from time import sleep, time
from simple_pid import PID

# start here tomorrow

class MobSoft:
    def __init__(self): 
        # Run main function
        sp = serial.Serial('/dev/ttyUSB0', 9600)
        self.run(sp)

    def dist(self, a,b):
        return math.sqrt((a*a)+(b*b))

    def get_x_rotation(self, x,y,z):
        radians = math.atan(x / self.dist(y,z))
        return math.degrees(radians)

    def get_y_rotation(self, x,y,z):
        radians = math.atan(y / self.dist(x,z))
        return math.degrees(radians)
    
    def angle_command_multiple(self,ser,
                    angle_head,
                    angle_cam,
                    angle_left_shoulder,
                    angle_left_elbow,
                    angle_left_hand,
                    angle_left_hip,
                    angle_left_thigh,
                    angle_left_knee,
                    angle_left_foot,
                    angle_right_shoulder,
                    angle_right_elbow,
                    angle_right_hand,
                    angle_right_hip,
                    angle_right_thigh,
                    angle_right_knee,
                    angle_right_foot,
                    ):
        """ Send angle command to the robot motors"""
        set_head = int(angle_head * 11.11 + 501)
        set_cam = int(angle_cam * 11.11 + 501)
        set_l_shoulder = int(angle_left_shoulder * 11.11 + 501)
        set_l_elbow = int(angle_left_elbow * 11.11 + 501)
        set_l_hand = int(angle_left_hand * 11.11 + 501)
        set_l_hip = int(angle_left_hip * 11.11 + 501)
        set_l_thigh = int(angle_left_thigh * 11.11 + 501)
        set_l_knee = int(angle_left_knee * 11.11 + 501)
        set_l_foot = int(angle_left_foot * 11.11 + 501)
        set_r_shoulder = int(angle_right_shoulder * 11.11 + 501)
        set_r_elbow = int(angle_right_elbow * 11.11 + 501)
        set_r_hand = int(angle_right_hand * 11.11 + 501)
        set_r_hip = int(angle_right_hip * 11.11 + 501)
        set_r_thigh = int(angle_right_thigh * 11.11 + 501)
        set_r_knee = int(angle_right_knee * 11.11 + 501)
        set_r_foot = int(angle_right_foot * 11.11 + 501)
        head = '14' #14
        cam  = '15' # 15
        left_shoulder  = '16' # 16
        left_elbow     = '17' # 17
        left_hand      = '18' # 18
        left_hip       = '28' # 28
        left_thigh     = '29' # 29
        left_knee      = '30' # 30
        left_foot      = '31' # 31
        right_shoulder = '20' # 20
        right_elbow    = '21' # 21
        right_hand     = '22' # 22
        right_hip      = '24' # 24
        right_thigh    = '25' # 25
        right_knee     = '26' # 26
        right_foot     = '27' # 27
        
        #print(str(set_value))
        if 499 < set_head < 2501 and 499 < set_cam < 2501 and 499 < set_l_shoulder < 2501 and 499 < set_l_elbow < 2501 and 499 < set_l_hand < 2501 and 499 < set_l_hip < 2501 and 499 < set_l_thigh < 2501 and 499 < set_l_knee < 2501 and 499 < set_l_thigh < 2501 and 499 < set_l_knee < 2501 and 499 < set_l_foot < 2501 and 499 < set_r_shoulder < 2501 and 499 < set_r_elbow < 2501 and 499 < set_r_hand < 2501 and 499 < set_r_hip < 2501 and 499 < set_r_thigh < 2501 and 499 < set_r_knee < 2501 and 499 < set_r_foot < 2501 :
            command = '#'+head+' P'+str(set_head)+' #'+cam+' P'+str(set_cam)+' #'+left_shoulder+' P'+str(set_l_shoulder)+' #'+left_elbow+' P'+str(set_l_elbow)+' #'+left_hip+' P'+str(set_l_hip)+' #'+left_thigh+' P'+str(set_l_thigh)+' #'+left_knee+' P'+str(set_l_knee)+' #'+left_foot+' P'+str(set_l_foot)+' #'+right_shoulder+' P'+str(set_r_shoulder)+' #'+right_elbow+' P'+str(set_r_elbow)+' #'+right_hand+' P'+str(set_r_hand)+' #'+right_hip+' P'+str(set_r_hip)+' #'+right_thigh+' P'+str(set_r_thigh)+' #'+right_knee+' P'+str(set_r_knee)+' #'+right_foot+' P'+str(set_r_foot)+' T1000 \r'
            print(command)
            ser.write(command.encode())
            #ser.flush()
        else:
            print("Out of Bounds")
    
    def stand(self, sp):
        print("standing")
        """These are the Default values for PIDs"""
        # PID setpoint head
        head_out = 90
        cam_out  = 120
        
        # PID setpoint left Side
        left_shoulder_out = 160
        left_elbow_out    = 130
        left_hand_out     = 90
        
        left_hip_out      = 100
        left_thigh_out    = 110
        left_knee_out     = 60
        left_foot_out     = 90
        
        # PID setopint right side
        right_shoulder_out = 20
        right_elbow_out    = 90
        right_hand_out     = 90
        
        right_hip_out      = 90
        right_thigh_out    = 100
        right_knee_out     = 48
        right_foot_out     = 100
        self.angle_command_multiple(sp, head_out, cam_out, left_shoulder_out, left_elbow_out, left_hand_out, left_hip_out, left_thigh_out, left_knee_out, left_foot_out, right_shoulder_out, right_elbow_out, right_hand_out, right_hip_out, right_thigh_out, right_knee_out, right_foot_out,)
     
    def stage_1_1(self, sp):
        print("stage one")
        """These are the Default values for PIDs"""
        
        # PID setpoint head
        head_out = 90
        cam_out  = 120
        
        # PID setpoint left Side
        left_shoulder_out = 160
        left_elbow_out    = 130
        left_hand_out     = 90
        
        left_hip_out      = 98
        left_thigh_out    = 97
        left_knee_out     = 60
        left_foot_out     = 79
        
        # PID setopint right side
        right_shoulder_out = 20
        right_elbow_out    = 90
        right_hand_out     = 90
        
        right_hip_out      = 90
        right_thigh_out    = 100
        right_knee_out     = 38
        right_foot_out     = 95
        self.angle_command_multiple(sp, head_out, cam_out, left_shoulder_out, left_elbow_out, left_hand_out, left_hip_out, left_thigh_out, left_knee_out, left_foot_out, right_shoulder_out, right_elbow_out, right_hand_out, right_hip_out, right_thigh_out, right_knee_out, right_foot_out,)
    
    def stage_1_2(self, sp):
        print("stage one")
        """These are the Default values for PIDs"""
        
        
        # PID setpoint head
        head_out = 90
        cam_out  = 120
        
        # PID setpoint left Side
        left_shoulder_out = 160
        left_elbow_out    = 130
        left_hand_out     = 90
        
        left_hip_out      = 98
        left_thigh_out    = 97
        left_knee_out     = 60
        left_foot_out     = 79
        
        # PID setopint right side
        right_shoulder_out = 20
        right_elbow_out    = 90
        right_hand_out     = 90
        
        right_hip_out      = 90
        right_thigh_out    = 110
        right_knee_out     = 40
        right_foot_out     = 95
        self.angle_command_multiple(sp, head_out, cam_out, left_shoulder_out, left_elbow_out, left_hand_out, left_hip_out, left_thigh_out, left_knee_out, left_foot_out, right_shoulder_out, right_elbow_out, right_hand_out, right_hip_out, right_thigh_out, right_knee_out, right_foot_out,)
    
    def stage_2_1(self, sp):
        print("stage two")
        """These are the Default values for PIDs"""
        # PID setpoint head
        head_out = 90
        cam_out  = 120
        
        # PID setpoint left Side
        left_shoulder_out = 160
        left_elbow_out    = 110
        left_hand_out     = 90
        
        left_hip_out      = 117
        left_thigh_out    = 100
        left_knee_out     = 60
        left_foot_out     = 110
        
        # PID setopint right side
        right_shoulder_out = 20
        right_elbow_out    = 90
        right_hand_out     = 90
        
        right_hip_out      = 100
        right_thigh_out    = 110
        right_knee_out     = 57
        right_foot_out     = 118
        self.angle_command_multiple(sp, head_out, cam_out, left_shoulder_out, left_elbow_out, left_hand_out, left_hip_out, left_thigh_out, left_knee_out, left_foot_out, right_shoulder_out, right_elbow_out, right_hand_out, right_hip_out, right_thigh_out, right_knee_out, right_foot_out,)
    
    def stage_2_2(self, sp):
        print("stage two")
        """These are the Default values for PIDs"""
        # PID setpoint head
        head_out = 90
        cam_out  = 120
        
        # PID setpoint left Side
        left_shoulder_out = 160
        left_elbow_out    = 110
        left_hand_out     = 90
        
        left_hip_out      = 117
        left_thigh_out    = 70
        left_knee_out     = 40
        left_foot_out     = 110
        
        # PID setopint right side
        right_shoulder_out = 20
        right_elbow_out    = 90
        right_hand_out     = 90
        
        right_hip_out      = 100
        right_thigh_out    = 115
        right_knee_out     = 57
        right_foot_out     = 118
        self.angle_command_multiple(sp, head_out, cam_out, left_shoulder_out, left_elbow_out, left_hand_out, left_hip_out, left_thigh_out, left_knee_out, left_foot_out, right_shoulder_out, right_elbow_out, right_hand_out, right_hip_out, right_thigh_out, right_knee_out, right_foot_out,)


    def run(self, sp):

        # SetMotors to initial position
        #sp.write('#14 P1500 #15 P1500 #16 P1500 #17 P1600 #18 P1500 #19 P1700 #20 P1600 #21 P1600 #22 P1500 #23 P1600 #24 P1600 #25 P1750 #26 P1500 #27 P1700 #28 P1500 #29 P1600 #30 P1500 #31 P1500 T1000 \r'.encode())
        i = 0        
        while True:
            try:
                self.stand(sp)
                sleep(3)
                self.stage_1_1(sp)
                sleep(3)
                self.stage_1_2(sp)
                sleep(2)
                self.stand(sp)
                sleep(5)
                self.stage_2_1(sp)
                sleep(3)
                self.stage_2_2(sp)
                sleep(3)
                self.stand(sp)
                sleep(5)
                i = i +1
                print(i)
                      
            except Exception as e:
                print("ERROR, ", e)
                

if __name__ == '__main__':
    MobSoft()



