#!/usr/bin/python3.7
import smbus
import serial
import sys
from time import sleep
from simple_pid import PID
from mpu6050 import mpu6050

class MobSoft:
    def __init__(self):
        sp = serial.Serial('/dev/ttyUSB0', 9600)
        self.sensor = mpu6050(0x68)
        self.pid = PID(1, 0.05, 0, setpoint=0)
        self.pid.sample_time = 0.10
        self.run(sp)

    def run(self, sp):
        # Value pid trying to reach
        self.pid.setpoint = 0
        # SetMotors to initial position
        sp.write('#14 P1500 #15 P1500 #16 P1500 #17 P1600 #18 P1500 #19 P1700 #20 P1600 #21 P1600 #21 P1600 #22 P1500 #23 P1600 #24 P1600 #25 P1750 #26 P1500 #27 P1640 #28 P1500 #29 P1600 #30 P1500 #31 P1500 T1000 \r')

        while True:
            try:
                # data from MPU6050
                accel_data = self.sensor.get_accel_data()
                gyro_data  = self.sensor.get_gyro_data()
                # unpacking gyroscope dictionary values               
                x_data = gyro_data.get('x')
                y_data = gyro_data.get('y')
                print("x-Value: "+str(round(x_data)))
                print("y-Value: "+str(round(y_data)))

                # PID infusion 
                self.pid.setpoint = 0               # value the pid is trying to reach
                control_output_x = self.pid(x_data) # feed x into the pid controller

                print("Control x: "+str(round(control_output_x, 3)))
                # print("accelermeter  : "+str(accel_data))
                # print("gryroscope    : "+str(gyro_data)) 
                
                # Test on head motor to see if automatic config
                if control_output_x < 0:
                    sp.write('#14 P1600 T1000 \r')
                if control_output_x > 0:
                    sp.write('#14 P1400 T1000 \r')
                if control_output_x == 0:
                    sp.write('#14 P1500 T1000 \r')
                sleep(1)
            except Exception as e:
                print("ERROR, ", e)


if __name__ == '__main__':
    MobSoft()
