#!/usr/bin/python3.7
import smbus
import serial
import sys
import math
from time import sleep, time
from simple_pid import PID


class MobSoft:
    def __init__(self):
        sp = serial.Serial('/dev/ttyUSB0', 9600)
        # PID 
        self.pid_x = PID(0.97, 0.002, 0.2, setpoint=90) # P controler at 50% change the I and D later
        self.pid_x.sample_time = 0.10
        self.pid_y = PID(0.97, 0.002, 0.2, setpoint=90) # P controller at 50% change the I and D later
        self.pid_y.sample_time = 0.10
        # Motors
        self.motor_x = 90
        self.motor_y = 90
        # MPU6050
        self.PWR_MGMT_1 = 0x6B
        self.Device_Address = 0x68
        self.SMPLRT_DIV = 0x19
        self.CONFIG = 0x1A
        self.GYRO_CONFIG = 0x1B
        self.INT_ENABLE = 0x38
        self.ACCEL_XOUT_H = 0x3B
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F
        self.GYRO_XOUT_H = 0x43
        self.GYRO_YOUT_H = 0x45
        self.GYRO_ZOUT_H = 0x47
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
            print(command)
            ser.write(command)
            ser.flush()
        else:
            print("Out of Bounds")

    def left_leg_control(self, ser, x, y):
        x = int(x * 11.11 - 501)
        y = int(y * 11.11 - 501)
        print("x = "+str(x))
        print("y = "+str(y))
        if 499 < x < 2501 or 499 < y < 2501:
            command = '#21 P'+str(x)+' #22 P'+str(y)+' #23 P1600 #24 P1600 #25 P1750 T500 \r'
            print(command)
            ser.write(command)
            ser.flush()

    def right_leg_control(self, ser, x, y):
        x = int(x * 11.11 + 501)
        y = int(y * 11.11 + 501)
        print("x = "+str(x))
        print("y = "+str(y))
        if 499 < x < 2501 or 499 < y < 2501:
            command = '#16 P'+str(x)+' #17 P'+str(y)+' #18 P1500 #19 P1700 #20 P1600 T500 \r'
            print(command)
            ser.write(command)
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
        sp.write('#14 P1500 #15 P1500 #16 P1500 #17 P1600 #18 P1500 #19 P1700 #20 P1600 #21 P1600 #22 P1500 #23 P1600 #24 P1600 #25 P1750 #26 P1500 #27 P1640 #28 P1500 #29 P1600 #30 P1500 #31 P1500 T1000 \r')

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

                #print("Gryroscope x: %.2f " % Gx)
                #print("Gryroscope y: %.2f " % Gy)
                #print("Gryroscope z: %.2f " % Gz)
    
                #print("Accel x : %.2f " % Ax)
                #print("Accel y : %.2f " % Ay)
                #print("Accel z : %.2f " % Az)
                
                actual_angle_x = int(self.get_x_rotation(Ax, Ay, Az))
                actual_angle_y = int(self.get_y_rotation(Ax, Ay, Az))
                
                print("Rotations")
                print("X Rotation: "+str(actual_angle_x))
                print("Y Rotation: "+str(actual_angle_y))
                
                # Difference
                diff_x = set_angle_x - actual_angle_x
                diff_y = set_angle_y - actual_angle_y
                print("Diff")
                print("Difference in x: "+str(diff_x))
                print("Difference in y: "+str(diff_y))

                # PID x control
                control_output_x = int(self.pid_x(diff_x)) # feed x error into the pid controller
                self.pid_x.setpoint = 90 
                
                # PID y control
                control_output_y = int(self.pid_y(diff_y)) # feed y error into the pid controller
                self.pid_y.setpoint = 90

                print("PID ouputs")
                print("PID X    : "+str(control_output_x))
                print("PID Y    : "+str(control_output_y))
                
                # PID control to hip motor
                self.right_leg_control(sp, control_output_x, control_output_y)
                self.left_leg_control(sp, control_output_x, control_output_y)

                sleep(1)
            except Exception as e:
                print("ERROR, ", e)

if __name__ == '__main__':
    MobSoft()
