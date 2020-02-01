#!/usr/bin/python3.7
import smbus
import serial
import sys
import math
import matplotlib.pyplot as plt
from time import sleep, time
from simple_pid import PID


class MobSoft:
    def __init__(self):
        sp = serial.Serial('/dev/ttyUSB0', 9600)
        
        # PID RIGHT LEG
        self.pid_left_x = PID(0.99, 0.0, 0.1, setpoint=100)
        self.pid_left_x.sample_time = 0.01
        self.pid_left_y = PID(0.99, 0.0, 0.1, setpoint=90)
        self.pid_left_y.sample_time = 0.01
        # PID RIGHT LEG
        self.pid_right_x = PID(0.99, 0.0, 0.1, setpoint=100)
        self.pid_right_x.sample_time = 0.01
        self.pid_right_y = PID(0.99, 0.0, 0.1, setpoint=100)
        self.pid_right_y.sample_time = 0.01
        # Motors
        self.motor_x = 90
        self.motor_y = 90
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
        print(str(set_value))
        if 499 < set_value < 2501:
            command = '#'+str(joint)+' P'+str(set_value)+' T1000 \r'
            #print(command)
            ser.write(command.encode())
            ser.flush()
        else:
            print("Out of Bounds")

    def left_leg_control(self, ser, x, y):
        if 98 < x < 120 or 70 < y < 100:
            x = int(x * 11.11 + 501)
            y = int(y * 11.11 + 501)
            #print("x = "+str(x))
            #print("y = "+str(y))
            if 1590 < x < 2000 or 499 < y < 2501:
                command = '#21 P'+str(x)+' #22 P'+str(y)+' #23 P1600 #24 P1600 #25 P1750 T400 \r'
                #print(command)
                ser.write(command.encode())
                ser.flush()
        else:   
                command = '#21 P1600 #22 P1500 #23 P1600 #24 P1600 #25 P1750 T400 \r'
                ser.write(command.encode())
                ser.flush()

    def right_leg_control(self, ser, x, y):
        if 80 < x < 95 or 70 < y < 100:
            x = int(x * 11.11 + 501)
            y = int(y * 11.11 + 501)
            #print("x = "+str(x))
            #print("y = "+str(y))
            if 499 < x < 1580 or 499 < y < 2501:
                command = '#16 P'+str(x)+' #17 P'+str(y)+' #18 P1500 #19 P1700 #20 P1600 T400 \r'
                #print(command)
                ser.write(command.encode())
                ser.flush()
        else:   
                command = '#16 P1500 #17 P1600 #18 P1500 #19 P1700 #20 P1600 T400 \r'
                ser.write(command.encode())
                ser.flush()
            
    def run(self, sp):
        # init position of motors
        self.mpu_init()
        
        actual_angle_x = 0
        actual_angle_x = 0

        set_angle_x = 0
        set_angle_y = 0
        
        diff_x = 0
        diff_y = 0

        # SetMotors to initial position
        sp.write('#14 P1500 #15 P1500 #16 P1500 #17 P1600 #18 P1500 #19 P1700 #20 P1600 #21 P1600 #22 P1500 #23 P1600 #24 P1600 #25 P1750 #26 P1500 #27 P1700 #28 P1500 #29 P1600 #30 P1500 #31 P1500 T1000 \r'.encode())
        
        # PID RIGHT LEG
        #self.pid_left_x.output_limits = (0,100)
        #self.pid_left_y.output_limits = (0,90)
        # PID RIGHT LEG
        #self.pid_right_x.output_limits = (0,90)
        #self.pid_right_y.output_limits = (0,90)
                
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
                
                # Difference
                diff_x = set_angle_x - actual_angle_x
                diff_y = set_angle_y - actual_angle_y

                # PID right leg
                right_output_x = int(self.pid_right_x(-diff_x)) # feed x error into the pid controller
                #self.pid_right_x.setpoint = 90 
                right_output_y = int(self.pid_right_y(diff_y)) # feed y error into the pid controller
                #self.pid_right_y.setpoint = 90          
                self.right_leg_control(sp, right_output_x, right_output_y)
                
                # PID left leg
                left_output_x = int(self.pid_left_x(-diff_x)) # feed x error into the pid controller
                #self.pid_left_x.setpoint = 100 
                left_output_y = int(self.pid_left_y(-diff_y)) # feed y error into the pid controller
                #self.pid_left_y.setpoint = 90          
                self.left_leg_control(sp, left_output_x, left_output_y)
                
                # PID RIGHT LEG pid variables
                plx, ilx, dlx = self.pid_left_x.components
                ply, ily, dly = self.pid_left_y.components
                # PID RIGHT LEG pid variables
                prx, irx, drx = self.pid_right_x.components
                pry, iry, dry = self.pid_right_y.components
                
                # print every second
                #print("Rotations")
                #print("X Rotation: "+str(actual_angle_x))
                #print("Y Rotation: "+str(actual_angle_y))
                #print("Diff")
                #print("Difference in x: "+str(diff_x))
                #print("Difference in y: "+str(diff_y))
                #print("PID ouputs for right leg")
                #print("px = "+str(plx)+" ix = "+str(ilx)+" dx = "+str(dlx))
                #print("py = "+str(ply)+" iy = "+str(ily)+" dy = "+str(dly))
                #print("PID X    : "+str(right_output_x))
                #print("PID Y    : "+str(right_output_y))
                #print("PID ouputs for left leg")
                #print("px = "+str(prx)+" ix = "+str(irx)+" dx = "+str(drx))
                #print("py = "+str(pry)+" iy = "+str(iry)+" dy = "+str(dry))
                #print("PID X    : "+str(left_output_x))
                #print("PID Y    : "+str(left_output_y))
                
                
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

                # Uncomment sleep to debug
                #sleep(1)      
            except Exception as e:
                print("ERROR, ", e)

if __name__ == '__main__':
    MobSoft()
