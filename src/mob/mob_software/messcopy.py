#!/usr/bin/python3.7
import smbus
import serial
import sys
from time import sleep
from simple_pid import PID

class Gyroscope:
    def __init__(self):
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
        self.pid_x = PID(1, 0.09, 1, setpoint=0)
        self.pid_y = PID(1, 0.09, 0.1, setpoint=0)
        self.pid_x.sample_time = 0.01
        self.pid_y.sample_time = 0.01
        self.run()

    def setup(self):
        # Write to sample rate register
        self.bus.write_byte_data(self.Device_Address, self.SMPLRT_DIV, 7)

        # Write to power management register
        self.bus.write_byte_data(self.Device_Address, self.PWR_MGMT_1, 1)

        # Write to Gyro configuration register
        self.bus.write_byte_data(self.Device_Address, self.GYRO_CONFIG, 24)

        # Write to interrupt enable register
        self.bus.write_byte_data(self.Device_Address, self.INT_ENABLE, 1)

    def read_raw_data(self, addr):
        # Accelero and Gyro Value are 16-bits
        high = self.bus.read_byte_data(self.Device_Address, addr)
        low = self.bus.read_byte_data(self.Device_Address, addr+1)

        # Concatenate higher and lower value
        value = ((high << 8) | low)

        # Get signed value from MPU6050
        if value > 32768:
            value = value - 65536
        return value

    def run(self):
        while True:
            try:
                self.setup()
                # Read Accelerometer raw value
                acc_x = self.read_raw_data(self.ACCEL_XOUT_H)
                acc_y = self.read_raw_data(self.ACCEL_YOUT_H)
                acc_z = self.read_raw_data(self.ACCEL_ZOUT_H)

                # Read Gyroscope raw value
                gyro_x = self.read_raw_data(self.GYRO_XOUT_H)
                gyro_y = self.read_raw_data(self.GYRO_YOUT_H)
                gyro_z = self.read_raw_data(self.GYRO_ZOUT_H)

                # Full Scale range +/- 250 degree/C as per senstivity scale factor
                accel_x = acc_x/16384.0
                accel_y = acc_y/16384.0
                accel_z = acc_z/16384.0
            
                control_x = self.pid_x(accel_x)
                control_y = self.pid_y(accel_y) 

                gyroscope_x = gyro_x/131.0
                gyroscope_y = gyro_y/131.0
                gyroscope_z = gyro_z/131.0
                
                print("\n")
                print("Control x: "+str(round(control_x, 3)))
                print("Control y: "+str(round(control_y, 3)))
                print("accel_x  : "+str(round(accel_x, 3)))
                print("accel_y  : "+str(round(accel_y, 3)))
            
                sleep(2)
            except Exception as e:
                print("ERROR, ", e)


if __name__ == '__main__':
    Gyroscope()
