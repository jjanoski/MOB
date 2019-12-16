#!/usr/bin/env python

import smbus
from time import sleep


class gyroscope:
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
        self.driver()

    def mpu_init(self):
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

    def driver(self):
        try:
            self.mpu_init()
            count = 0

            while True:
                # Read Accelerometer raw value
                acc_x = self.read_raw_data(self.ACCEL_XOUT_H)
                acc_y = self.read_raw_data(self.ACCEL_YOUT_H)
                acc_z = self.read_raw_data(self.ACCEL_ZOUT_H)

                # Read Gyroscope raw value
                gyro_x = self.read_raw_data(self.GYRO_XOUT_H)
                gyro_y = self.read_raw_data(self.GYRO_YOUT_H)
                gyro_z = self.read_raw_data(self.GYRO_ZOUT_H)

                # Full Scale range +/- 250 degree/C as per senstivity scale factor
                Ax = acc_x/16384.0
                Ay = acc_y/16384.0
                Az = acc_z/16384.0

                Gx = gyro_x/131.0
                Gy = gyro_y/131.0
                Gz = gyro_z/131.0
                print("Readout: "+str(count))
                print("Gryroscope x: %.2f " % Gx)
                print("Gryroscope y: %.2f " % Gy)
                print("Gryroscope z: %.2f " % Gz)
    
                print("Accel x : %.2f " % Ax)
                print("Accel y : %.2f " % Ay)
                print("Accel z : %.2f " % Az)
                count += 1
                sleep(2)
                return [Ax, Ay, Az, Gx, Gy, Gz]
        except Exception as e:
            print("ERROR, ", e)


if __name__ == '__main__':
    gyroscope()
