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

        sp = serial.Serial('/dev/ttyUSB0', 9600)
        # PID head
        self.head          = PID(0.99, 0.0, 0.90, setpoint= 90)     #14  < right -- setpiont ++ left >
        self.cam           = PID(0.99, 0.0, 0.90, setpoint= 120)     #15  < up    -- setpiont ++ down >
        # PID Left Side
        self.left_shoulder = PID(0.99, 0.0, 0.90, setpoint= 160)     # 29 < foward   -- setpiont ++ backward  >
        self.left_elbow    = PID(0.99, 0.0, 0.90, setpoint= 110)   # 30 < left     -- setpiont ++ right >
        self.left_hand     = PID(0.99, 0.0, 0.90, setpoint= 90)     # 31 < right    -- setpiont ++ left  >
        self.left_hip      = PID(0.99, 0.0, 0.90, setpoint= 100) # 21 < right    -- setpiont ++ left  >
        self.left_thigh    = PID(0.99, 0.0, 0.90, setpoint= 90)   # 22 < forward  -- setpoint ++ backward >
        self.left_knee     = PID(0.99, 0.0, 0.90, setpoint= 60) # 23 < backward -- setpoint ++ forward >
        self.left_foot     = PID(0.99, 0.0, 0.90, setpoint= 90)   # 25 < left     -- setpoint ++ right >
        # PID Right Side
        self.right_shoulder = PID(0.99, 0.0, 0.90, setpoint= 20)     # 28 < backward -- setpoint ++ forward >
        self.right_elbow    = PID(0.99, 0.0, 0.90, setpoint= 90) # 27 < left     -- setpoint ++ right >
        self.right_hand     = PID(0.99, 0.0, 0.90, setpoint= 90)     # 26 < left     -- setpoint ++ right >
        self.right_hip      = PID(0.99, 0.0, 0.90, setpoint= 90)     # 16 < right    -- setpoint -- left >
        self.right_thigh    = PID(0.99, 0.0, 0.90, setpoint= 100) # 17 < backward -- setpoint ++ forward >
        self.right_knee     = PID(0.99, 0.0, 0.90, setpoint= 50)     # 18 < forward  -- setpoint ++ backward >
        self.right_foot     = PID(0.99, 0.0, 0.90, setpoint= 100)     # 20 < left     -- setpoint ++ right >
        # Initial values
        self.default_init()

        # Run main function
        self.run(sp)

    def dist(self, a,b):
        return math.sqrt((a*a)+(b*b))

    def get_x_rotation(self, x,y,z):
        radians = math.atan(x / self.dist(y,z))
        return math.degrees(radians)

    def get_y_rotation(self, x,y,z):
        radians = math.atan(y / self.dist(x,z))
        return math.degrees(radians)

    def angle_command(self,ser, joint, angle):
        """ Send angle command to the robot motors"""
        set_value = int(angle * 11.11 + 501)
        #print(str(set_value))
        if 499 < set_value < 2501:
            command = '#'+str(joint)+' P'+str(set_value)+' T1000 \r'
            #print(command)
            ser.write(command.encode())
            ser.flush()
        else:
            print("Out of Bounds")
        
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
            
    def left_leg_control(self, ser, x, y, t):
        if 100 <= x < 120 or 70 < y < 100:
            x = int(x * 11.11 + 501)
            y = int(y * 11.11 + 501)
            #print("x = "+str(x))
            #print("y = "+str(y))
            if 1590 < x < 2000 or 499 < y < 2501:
                command = '#21 P'+str(x)+' #22 P'+str(y)+' T'+str(t)+' \r'
                #print(command)
                ser.write(command.encode())
                #ser.flush()

    def right_leg_control(self, ser, x, y, t):
        if 80 < x < 95 or 70 < y < 100:
            x = int(x * 11.11 + 501)
            y = int(y * 11.11 + 501)
            #print("x = "+str(x))
            #print("y = "+str(y))
            if 499 < x < 1580 or 499 < y < 2501:
                command = '#16 P'+str(x)+' #17 P'+str(y)+' T'+str(t)+' \r'
                #print(command)
                ser.write(command.encode())
                #ser.flush()
                
    def default_init(self):
        # Initial values for motor PID's
        self.head_init = 0
        self.cam_init  = 0
        # PID Left Side
        self.left_shoulder_init =0
        self.left_elbow_init    =0
        self.left_hand_init     =0
        self.left_hip_init      =0
        self.left_thigh_init    =0
        self.left_knee_init     =0
        self.left_foot_init     =0
        # PID Right Side
        self.right_shoulder_init = 0
        self.right_elbow_init    = 0
        self.right_hand_init     = 0
        self.right_hip_init      = 0
        self.right_thigh_init    = 0
        self.right_knee_init     = 0
        self.right_foot_init     = 0
        
    def stand(self, sp):
        """These are the Default values for PIDs"""
        # PID setpoint head
        self.head.setpoint = 90
        self.cam.setpoint  = 120
        
        # PID setpoint left Side
        self.left_shoulder.setpoint = 160
        self.left_elbow.setpoint    = 110
        self.left_hand.setpoint     = 90
        self.left_hip.setpoint      = 100
        self.left_thigh.setpoint    = 90
        self.left_knee.setpoint     = 60
        self.left_foot.setpoint     = 90
        
        # PID setopint right side
        self.right_shoulder.setpoint = 20
        self.right_elbow.setpoint    = 90
        self.right_hand.setpoint     = 90
        self.right_hip.setpoint      = 90
        self.right_thigh.setpoint    = 100
        self.right_knee.setpoint     = 50
        self.right_foot.setpoint     = 100
    
        # Head PIDS
        head_out = self.head(self.head_init)
        cam_out  = self.cam(self.cam_init)
        
        # PID Left Side
        left_shoulder_out = self.left_shoulder(self.left_shoulder_init)
        left_elbow_out = self.left_elbow(self.left_elbow_init)
        left_hand_out  = self.left_hand(self.left_hand_init)
        left_hip_out = self.left_hip(self.left_hip_init)
        left_thigh_out = self.left_thigh(self.left_thigh_init)
        left_knee_out  = self.left_knee(self.left_knee_init)
        left_foot_out  = self.left_foot(self.left_foot_init)
        
        # PID Right Side
        right_shoulder_out = self.right_shoulder(self.right_shoulder_init)
        right_elbow_out = self.right_elbow(self.right_elbow_init)
        right_hand_out  = self.right_hand(self.right_hand_init)
        right_hip_out = self.right_hip(self.right_hip_init)
        right_thigh_out = self.right_thigh(self.right_thigh_init)
        right_knee_out  = self.right_knee(self.right_knee_init)
        right_foot_out  = self.right_foot(self.right_foot_init)
        self.angle_command_multiple(sp, head_out, cam_out, left_shoulder_out, left_elbow_out, left_hand_out, left_hip_out, left_thigh_out, left_knee_out, left_foot_out, right_shoulder_out, right_elbow_out, right_hand_out, right_hip_out, right_thigh_out, right_knee_out, right_foot_out,)
    
    def stage_one(self, sp):
        # PID setpoint head
        self.head.setpoint = 90
        self.cam.setpoint  = 120
        
        # PID setpoint left Side
        self.left_shoulder.setpoint = 160
        self.left_elbow.setpoint    = 110
        self.left_hand.setpoint     = 90
        self.left_hip.setpoint      = 100
        self.left_thigh.setpoint    = 90
        self.left_knee.setpoint     = 60
        self.left_foot.setpoint     = 90
        
        # PID setopint right side
        self.right_shoulder.setpoint = 20
        self.right_elbow.setpoint    = 90
        self.right_hand.setpoint     = 90
        self.right_hip.setpoint      = 90
        self.right_thigh.setpoint    = 100
        self.right_knee.setpoint     = 50
        self.right_foot.setpoint     = 100
    
        # Head PIDS
        head_out = self.head(self.head_init)
        cam_out  = self.cam(self.cam_init)
        
        # PID Left Side
        left_shoulder_out = self.left_shoulder(self.left_shoulder_init)
        left_elbow_out = self.left_elbow(self.left_elbow_init)
        left_hand_out  = self.left_hand(self.left_hand_init)
        left_hip_out = self.left_hip(self.left_hip_init)
        left_thigh_out = self.left_thigh(self.left_thigh_init)
        left_knee_out  = self.left_knee(self.left_knee_init)
        left_foot_out  = self.left_foot(self.left_foot_init)
        
        # PID Right Side
        right_shoulder_out = self.right_shoulder(self.right_shoulder_init)
        right_elbow_out = self.right_elbow(self.right_elbow_init)
        right_hand_out  = self.right_hand(self.right_hand_init)
        right_hip_out = self.right_hip(self.right_hip_init)
        right_thigh_out = self.right_thigh(self.right_thigh_init)
        right_knee_out  = self.right_knee(self.right_knee_init)
        right_foot_out  = self.right_foot(self.right_foot_init)
        self.angle_command_multiple(sp, head_out, cam_out, left_shoulder_out, left_elbow_out, left_hand_out, left_hip_out, left_thigh_out, left_knee_out, left_foot_out, right_shoulder_out, right_elbow_out, right_hand_out, right_hip_out, right_thigh_out, right_knee_out, right_foot_out,)
    
    def run(self, sp):

        # SetMotors to initial position
        sp.write('#14 P1500 #15 P1500 #16 P1500 #17 P1600 #18 P1500 #19 P1700 #20 P1600 #21 P1600 #22 P1500 #23 P1600 #24 P1600 #25 P1750 #26 P1500 #27 P1700 #28 P1500 #29 P1600 #30 P1500 #31 P1500 T1000 \r'.encode())
        i = 0        
        while True:
            try:

                self.stand(sp)
                sleep(3)
                self.stage_one(sp)
                sleep(5)
                self.stand(sp)
                sleep(5)
                i = i +1
                print(i)
                      
            except Exception as e:
                print("ERROR, ", e)
                

if __name__ == '__main__':
    MobSoft()

