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
        self.head          = PID(0.99, 0.0, 0.90, setpoint= 90) #14
        self.cam           = PID(0.99, 0.0, 0.90, setpoint= 90) #15
        # PID Left Side
        self.left_shoulder = PID(0.99, 0.0, 0.90, setpoint= 99) # 29
        self.left_elbow    = PID(0.99, 0.0, 0.90, setpoint= 76.5) # 30
        self.left_hand     = PID(0.99, 0.0, 0.90, setpoint= 90) # 31
        self.left_hip      = PID(0.99, 0.0, 0.90, setpoint= 103.51) # 21
        self.left_thigh    = PID(0.99, 0.0, 0.90, setpoint= 85.5) # 22
        self.left_knee     = PID(0.99, 0.0, 0.90, setpoint= 108.01) # 23
        self.left_ankle    = PID(0.99, 0.0, 0.90, setpoint= 99) # 24
        self.left_foot     = PID(0.99, 0.0, 0.90, setpoint= 94.5) # 25
        # PID Right Side
        self.right_shoulder = PID(0.99, 0.0, 0.90, setpoint= 90) # 28
        self.right_elbow    = PID(0.99, 0.0, 0.90, setpoint= 117.01) # 27
        self.right_hand     = PID(0.99, 0.0, 0.90, setpoint= 90) # 26
        self.right_hip      = PID(0.99, 0.0, 0.90, setpoint= 90) # 16
        self.right_thigh    = PID(0.99, 0.0, 0.90, setpoint= 103.51) # 17
        self.right_knee     = PID(0.99, 0.0, 0.90, setpoint= 99) # 18
        self.right_ankle    = PID(0.99, 0.0, 0.90, setpoint= 99) # 19
        self.right_foot     = PID(0.99, 0.0, 0.90, setpoint= 99) # 20
        # Initial values
        self.default_init()
        # MPU6050
        self.PWR_MGMT_1     = 0x6B
        self.Device_Address = 0x68
        self.SMPLRT_DIV     = 0x19
        self.CONFIG         = 0x1A
        self.GYRO_CONFIG    = 0x1B
        self.INT_ENABLE     = 0x38
        self.ACCEL_XOUT_H   = 0x3B
        self.ACCEL_YOUT_H   = 0x3D
        self.ACCEL_ZOUT_H   = 0x3F
        self.GYRO_XOUT_H    = 0x43
        self.GYRO_YOUT_H    = 0x45
        self.GYRO_ZOUT_H    = 0x47
        self.bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
       
        # Run main function
        self.run(sp)
        
    def default_init(self):
        # Initial values for motor PID's
        self.head_init = 0 #14
        self.cam_init  = 0 #15
        # PID Left Side
        self.left_shoulder_init =0 # 29
        self.left_elbow_init    =0 # 30
        self.left_hand_init     =0 # 31
        self.left_hip_init      =0 # 21
        self.left_thigh_init    =0 # 22
        self.left_knee_init     =0 # 23
        self.left_ankle_init    =0 # 24
        self.left_foot_init     =0 # 25
        # PID Right Side
        self.right_shoulder_init = 0 # 28
        self.right_elbow_init    = 0 # 27
        self.right_hand_init     = 0 # 26
        self.right_hip_init      = 0  # 16
        self.right_thigh_init    = 0 # 17
        self.right_knee_init     = 0 # 18
        self.right_ankle_init    = 0 # 19
        self.right_foot_init     = 0 # 20
        

    def mpu_init(self):
        """ Initializes the MPU6050"""
        # Write to sample rate register
        self.bus.write_byte_data(self.Device_Address, self.SMPLRT_DIV, 7)

        # Write to power management register
        self.bus.write_byte_data(self.Device_Address, self.PWR_MGMT_1, 1)

        # Write to Gyro configuration register
        self.bus.write_byte_data(self.Device_Address, self.GYRO_CONFIG, 24)

        # Write to interrupt enable register
        self.bus.write_byte_data(self.Device_Address, self.INT_ENABLE, 1)

    def read_raw_data(self, addr):
        """ Read raw data form the address within the MPU6050"""
        # Accelero and Gyro Value are 16-bits
        high = self.bus.read_byte_data(self.Device_Address, addr)
        low = self.bus.read_byte_data(self.Device_Address, addr+1)

        # Concatenate higher and lower value
        value = ((high << 8) | low)

        # Get signed value from MPU6050
        if value > 32768:
            value = value - 65536
        return value
    
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
                    head = 14, #14
                    cam  = 15, # 15
                    left_shoulder  = 29, # 29
                    left_elbow     = 30, # 30
                    left_hand      = 31, # 31
                    left_hip       = 21, # 21
                    left_thigh     = 22, # 22
                    left_knee      = 23, # 23
                    left_ankle     = 24, # 24
                    left_foot      = 25, # 25
                    right_shoulder = 28, # 28
                    right_elbow    = 27, # 27
                    right_hand     = 26, # 26
                    right_hip      = 16, # 16
                    right_thigh    = 17, # 17
                    right_knee     = 18, # 18
                    right_ankle    = 19, # 19
                    right_foot     = 20, # 20
                    angle_head           = 14, #14
                    angle_cam            = 15, # 15
                    angle_left_shoulder  = 29, # 29
                    angle_left_elbow     = 30, # 30
                    angle_left_hand      = 31, # 31
                    angle_left_hip       = 21, # 21
                    angle_left_thigh     = 22, # 22
                    angle_left_knee      = 23, # 23
                    angle_left_ankle     = 24, # 24
                    angle_left_foot      = 25, # 25
                    angle_right_shoulder = 26, # 26
                    angle_right_elbow    = 27, # 27
                    angle_right_hand     = 28, # 28
                    angle_right_hip      = 16, # 16
                    angle_right_thigh    = 17, # 17
                    angle_right_knee     = 18, # 18
                    angle_right_ankle    = 19, # 19
                    angle_right_foot     = 20, # 20
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
        set_l_ankle = int(angle_left_ankle * 11.11 + 501)
        set_l_foot = int(angle_left_foot * 11.11 + 501)
        set_r_shoulder = int(angle_right_shoulder * 11.11 + 501)
        set_r_elbow = int(angle_right_elbow * 11.11 + 501)
        set_r_hand = int(angle_right_hand * 11.11 + 501)
        set_r_hip = int(angle_right_hip * 11.11 + 501)
        set_r_thigh = int(angle_right_thigh * 11.11 + 501)
        set_r_knee = int(angle_right_knee * 11.11 + 501)
        set_r_ankle = int(angle_right_ankle * 11.11 + 501)
        set_r_foot = int(angle_right_foot * 11.11 + 501)
        
        #print(str(set_value))
        if 499 < set_head < 2501 and 499 < set_cam < 2501 and 499 < set_l_shoulder < 2501 and 499 < set_l_elbow < 2501 and 499 < set_l_hand < 2501 and 499 < set_l_hip < 2501 and 499 < set_l_thigh < 2501 and 499 < set_l_knee < 2501 and 499 < set_l_ankle < 2501 and 499 < set_l_thigh < 2501 and 499 < set_l_knee < 2501 and 499 < set_l_ankle < 2501 and 499 < set_l_foot < 2501 and 499 < set_r_shoulder < 2501 and 499 < set_r_elbow < 2501 and 499 < set_r_hand < 2501 and 499 < set_r_hip < 2501 and 499 < set_r_thigh < 2501 and 499 < set_r_knee < 2501 and 499 < set_r_ankle < 2501 and 499 < set_r_foot < 2501 :
            command = '#'+str(head)+' P'+str(set_head)+' #'+str(cam)+' P'+str(set_cam)+' #'+str(left_shoulder)+' P'+str(set_l_shoulder)+' #'+str(left_elbow)+' P'+str(set_l_elbow)+' #'+str(left_hip)+' P'+str(set_l_hip)+' #'+str(left_thigh)+' P'+str(set_l_thigh)+' #'+str(left_knee)+' P'+str(set_l_knee)+' #'+str(left_ankle)+' P'+str(set_l_ankle)+' #'+str(left_foot)+' P'+str(set_l_foot)+' #'+str(right_shoulder)+' P'+str(set_r_shoulder)+' #'+str(right_elbow)+' P'+str(set_r_elbow)+' #'+str(right_hand)+' P'+str(set_r_hand)+' #'+str(right_hip)+' P'+str(set_r_hip)+' #'+str(right_thigh)+' P'+str(set_r_thigh)+' #'+str(right_knee)+' P'+str(set_r_knee)+' #'+str(right_ankle)+' P'+str(set_r_ankle)+' #'+str(right_foot)+' P'+str(set_r_foot)+' T1000 \r'
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
        
    def stand(self, sp, angle_x, angle_y):
        # PID right leg
        right_hip_out = self.right_hip(self.right_hip_init) # feed x error into the pid controller
        right_thigh_out = self.right_thigh(self.right_thigh_init) # feed y error into the pid controller
        # update the inputs
        #self.right_hip_init   = angle_x
        #self.right_thigh_init = -angle_y
        
        # PID left leg
        left_hip_out = self.left_hip(self.left_hip_init) # feed x error into the pid controller
        left_thigh_out = self.left_thigh(self.left_thigh_init) # feed y error into the pid controller
        # update the inputs
        #self.left_hip_init   = angle_x
        #self.left_thigh_init = angle_y

        # Head PIDS
        head_out = self.head(self.head_init) #14
        cam_out  = self.cam(self.cam_init)   #15
        
        # PID Left Side
        left_shoulder_out = self.left_shoulder(self.left_shoulder_init) # 29
        left_elbow_out = self.left_elbow(self.left_elbow_init) # 30
        left_hand_out  = self.left_hand(self.left_hand_init)   # 31
        #left_hip_out = self.left_hip(self.left_hip_init) # 21
        #left_thigh_out = self.left_thigh(self.left_thigh_init) # 22
        left_knee_out  = self.left_knee(self.left_knee_init)   # 23
        left_ankle_out = self.left_ankle(self.left_ankle_init) # 24
        left_foot_out  = self.left_foot(self.left_foot_init)   # 25
        
        # PID Right Side
        right_shoulder_out = self.right_shoulder(self.right_shoulder_init) # 26
        right_elbow_out = self.right_elbow(self.right_elbow_init) # 27
        right_hand_out  = self.right_hand(self.right_hand_init)   # 28
        #right_hip_out = self.right_hip(self.right_hip_init) # 16
        #right_thigh_out = self.right_thigh(self.right_thigh_init) # 17
        right_knee_out  = self.right_knee(self.right_knee_init)   # 18
        right_ankle_out = self.right_ankle(self.right_ankle_init) # 19
        right_foot_out  = self.right_foot(self.right_foot_init)   # 20
        self.angle_command_multiple(sp, 14, 15, 29, 30,31,23,22,21,24,25,26,27,28,16,17,18,19,20, head_out, cam_out, left_shoulder_out, left_elbow_out, left_hand_out, left_hip_out, left_thigh_out, left_knee_out, left_ankle_out, left_foot_out, right_shoulder_out, right_elbow_out, right_hand_out, right_hip_out, right_thigh_out, right_knee_out, right_ankle_out, right_foot_out,)

    
    def mpu_values(self):
        # data from MPU6050
                # Read Accelerometer raw value
                acc_y = self.read_raw_data(self.ACCEL_YOUT_H)
                acc_x = self.read_raw_data(self.ACCEL_XOUT_H)
                acc_z = self.read_raw_data(self.ACCEL_ZOUT_H)

                # Read Gyroscope raw value
                gyro_y = self.read_raw_data(self.GYRO_YOUT_H)
                gyro_x = self.read_raw_data(self.GYRO_XOUT_H)
                gyro_z = self.read_raw_data(self.GYRO_ZOUT_H)

                # Full Scale range +/- 250 degree/C as per senstivity scale factor
                Ax = acc_x/16384.0
                Ay = acc_y/16384.0
                Az = acc_z/16384.0

                Gx = gyro_x/131.0
                Gy = gyro_y/131.0
                Gz = gyro_z/131.0

                # Gryoscop raw data
                #print("Gryroscope x: %.2f " % Gx)
                #print("Gryroscope y: %.2f " % Gy)
                #print("Gryroscope z: %.2f " % Gz)
    
                #print("Accel x : %.2f " % Ax)
                #print("Accel y : %.2f " % Ay)
                #print("Accel z : %.2f " % Az)

                # actual angle after calculation
                actual_angle_x = int(self.get_x_rotation(Ax, Ay, Az))
                actual_angle_y = int(self.get_y_rotation(Ax, Ay, Az))
                return [actual_angle_x, actual_angle_y]
            
    def run(self, sp):
        # init position of motors
        self.mpu_init()
        
        # start actual at desired
        actual_angle_x = 0
        actual_angle_x = 0

        # SetMotors to initial position
        #sp.write('#14 P1500 #15 P1500 #16 P1500 #17 P1600 #18 P1500 #19 P1700 #20 P1600 #21 P1600 #22 P1500 #23 P1600 #24 P1600 #25 P1750 #26 P1500 #27 P1700 #28 P1500 #29 P1600 #30 P1500 #31 P1500 T1000 \r'.encode())
                
        # Init plot
        fig = plt.figure(figsize=(8,11))
        fig.suptitle('PID', fontsize=16)
        #ax1 = fig.add_subplot(111) # show only one window
        
        # Uncomment these to show all PIDs
        ax1  = fig.add_subplot(321) # top right
        ax2  = fig.add_subplot(322) # top left
        ax3  = fig.add_subplot(323) # bottom right
        ax4  = fig.add_subplot(324) # bottom left
        ax5  = fig.add_subplot(325)
        ax6  = fig.add_subplot(326)
        
        ax1.set_title('Left Hip')
        ax2.set_title('Left Thigh')
        ax3.set_title('Right Hip')
        ax4.set_title('Right Thigh')
        ax5.set_title('Gyroscop Angle X')
        ax6.set_title('Gyroscop Angle Y')
        
        fig.show()
        i = 0
        x1 = []
        x2 = []
        x3 = []
        x4 = []
        x5 = []
        x6 = []
        
        pid_lx_line = []
        pid_ly_line = []
        pid_rx_line = []
        pid_ry_line = []
        gyro_x_line = []
        gyro_y_line = []
        
        while True:
            try:
                #Time calc
                time_current = time()            # previoud time
                time_prev    = time_current           # current Time
                elapsed_time = time_current - time_prev  # elapsed time

                actual_angle_x, actual_angle_y = self.mpu_values()
                
                # Range Control
                if -5 < actual_angle_x < 5:
                    actual_angle_x = 0
                    
                else:
                    actual_angle_x = actual_angle_x
                    
                if -5 < actual_angle_y <= 5:
                    actual_angle_y = 0
                else:
                    actual_angle_y = actual_angle_y
                    
                print("angle x = "+str(actual_angle_x))
                print("angle y = "+str(actual_angle_y))

                self.stand(sp, actual_angle_x, actual_angle_y)
                #self.stage_one(sp, actual_angle_x, actual_angle_y)
                # PID RIGHT LEG pid variables
                plx, ilx, dlx = self.left_hip.components
                ply, ily, dly = self.left_thigh.components
                # PID RIGHT LEG pid variables
                prx, irx, drx = self.right_hip.components
                pry, iry, dry = self.right_thigh.components
                
                # plotting stuff
                x1.append(i)
                x2.append(i)
                x3.append(i)
                x4.append(i)
                x5.append(i)
                x6.append(i)
                
                pid_lx_line.append([plx, ilx, dlx])
                pid_ly_line.append([ply, ily, dly])
                pid_rx_line.append([prx, irx, drx])
                pid_ry_line.append([pry, iry, dry])
                
                gyro_x_line.append(actual_angle_x)
                gyro_y_line.append(actual_angle_y)
                
                ax1.plot(x1, pid_lx_line, color= 'b')
                ax2.plot(x2, pid_ly_line, color= 'g')
                ax3.plot(x3, pid_rx_line, color= 'r')
                ax4.plot(x4, pid_ry_line, color= 'y')
                ax5.plot(x5, gyro_x_line, color= 'm')
                ax6.plot(x6, gyro_y_line, color= 'k')
                fig.canvas.draw()
                ax1.set_xlim(left= max(0, i-10), right=i+10)
                ax2.set_xlim(left= max(0, i-10), right=i+10)
                ax3.set_xlim(left= max(0, i-10), right=i+10)
                ax4.set_xlim(left= max(0, i-10), right=i+10)
                ax5.set_xlim(left= max(0, i-10), right=i+10)
                ax6.set_xlim(left= max(0, i-10), right=i+10)
                #sleep(0.1)
                i = i+1
                #sp.flush()
                
                # Uncomment sleep to debug
                #sleep(1)      
            except Exception as e:
                print("ERROR, ", e)

if __name__ == '__main__':
    MobSoft()
