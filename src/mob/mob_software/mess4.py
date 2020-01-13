#!/usr/bin/python3.7
import smbus
import serial
import sys
import math
from time import sleep, time
from simple_pid import PID
from mpu6050 import mpu6050

class MobSoft:
    def __init__(self):
        sp = serial.Serial('/dev/ttyUSB0', 9600)
        self.sensor = mpu6050(0x68)
        self.pid_x = PID(0.5, 0, 0, setpoint=90) # P controler at 50% change the I and D later
        self.pid_x.sample_time = 0.10
        self.pid_y = PID(0.5, 0, 0, setpoint=90) # P controller at 50% change the I and D later
        self.pid_y.sample_time = 0.10
        self.time = int(round(time() * 1000))
        self.run(sp)
    
    def angle_command(self,ser, joint, angle):
        set_value = int(angle * 11.11 + 501)
        print(str(set_value))
        command = '#'+str(joint)+' P'+str(set_value)+' T1000 \r'
        print(command)
        ser.write(command)
        ser.flush()

    def run(self, sp):
        # init position of motors
        right_hip   = 1500  # MOTOR 16
        right_thigh = 1600  # MOTOR 17
        left_hip    = 1600  # MOTOR 21
        left_thigh  = 1500  # MOTOR 22
        angle_x = 90
        angle_y = 90
        total_angle_x = 0
        total_angle_y = 0

        # SetMotors to initial position
        sp.write('#14 P1500 #15 P1500 #16 P'+str(right_hip)+' #17 P'+str(right_thigh)+' #18 P1500 #19 P1700 #20 P1600 #21 P'+str(left_hip)+' #22 P'+str(left_thigh)+' #23 P1600 #24 P1600 #25 P1750 #26 P1500 #27 P1640 #28 P1500 #29 P1600 #30 P1500 #31 P1500 T1000 \r')

        while True:
            try:
                #Time calc
                time_prev = self.time;              # get previoud time
                self.time = int(round(time()*1000)) # read time
                elapsed_time = self.time - time_prev  # elapsed time

                # data from MPU6050
                accel_data = self.sensor.get_accel_data(True)  # G units
                gyro_data  = self.sensor.get_gyro_data()
                print(str(accel_data))
                print(str(gyro_data))

                # unpacking gyroscope dictionary values 
                z_accel_data = accel_data.get('z')

                x_accel_data = accel_data.get('x')
                x_accel_data = math.degrees(math.atan(x_accel_data/math.sqrt(x_accel_data ** 2)+z_accel_data **2)) 
                x_gyro_data = gyro_data.get('x')
                total_angle_x = 0.98*(total_angle_x + x_gyro_data*elapsed_time)+0.02*x_accel_data
                # x_data = x_data*(180/3.14)

                y_accel_data = accel_data.get('y')
                y_accel_data = math.degrees(-1*math.atan(y_accel_data/math.sqrt(y_accel_data ** 2)+z_accel_data **2)) 
                y_gyro_data = gyro_data.get('y')
                total_angle_y = 0.98*(total_angle_y + y_gyro_data*elapsed_time)+0.02*y_accel_data
                # y_data = y_data*(180/3.14)
                
                # print("x-accel-angle: "+str(round(x_accel_data, 2)))
                # print("y-accel-angle: "+str(round(y_accel_data, 2)))
                print("x-axis: "+str(round(total_angle_x, 2)))
                print("y-axis: "+str(round(total_angle_y, 2)))
                
                total_angle_y = 0
                total_angle_x = 0
                # PID x control
                control_output_x = self.pid_x(180) # feed x error into the pid controller
                # print("Control x: "+str(round(control_output_x, 2)))
                
                # PID y control
                control_output_y = self.pid_y(180) # feed y error into the pid controller
                # print("Control y: "+str(round(control_output_y, 2))) 

                # X Control
                if control_output_x == 0:
                    pass

                if control_output_x < 0:
                    angle_x = angle_x + control_output_x
                    # print("Angle X: "+str(angle_x))
                    # self.angle_command(sp, 16, angle_x)
                
                if control_output_x > 0:
                    angle_x = angle_x - control_ouput_x
                    # print("Angle X: "+str(angle_x))
                    # self.angle_command(sp, 16, angle_x)
                
                # Y control
                if control_output_y == 0: 
                    pass 
               
                if control_output_y < 0:
                    angle_y = angle_y + control_output_y
                    # self.angle_command(sp, 21, angle_y)
                
                if control_output_y > 0:
                    angle_y = angle_y - control_output_y
                    # self.angle_command(sp, 21, angle_y)

                sleep(1)
            except Exception as e:
                print("ERROR, ", e)

if __name__ == '__main__':
    MobSoft()
