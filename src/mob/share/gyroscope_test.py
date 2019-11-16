#!/usr/bin/env python

import smbus
from time import sleep

PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

def MPU_init():
    # Write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    # Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

    # Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

    # Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    # Accelero and Gyro Value are 16-bits
    high = bus.read_byte_data(Device_Address, addr)
    low  = bus.read_byte_data(Device_Address, addr+1)

    # Concatenate higher and lower value
    value = ((high<<8) | low) 

    # Get signed value from MPU6050
    if(value > 32768):
        value = value - 65536
    
    return value


# ---- main execution ----
bus = smbus.SMBus(1) # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68
MPU_init()
count = 0
try:
    # main loop
    while True:
        # Read Accelerometer raw value
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)

        # Read Gyroscope raw value
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)

        # Full Scale range +/- 250 degree/C as per senstivity scale factor
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0

        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0
        print("Readout: "+str(count))
        print("Gryroscope x: %.2f \n" %Gx)
        print("Gryroscope y: %.2f \n" %Gy)
        print("Gryroscope z: %.2f \n" %Gz)
    
        print("Accel x : %.2f \n" %Ax)
        print("Accel y : %.2f \n" %Ay)
        print("Accel z : %.2f \n" %Az)
        count +=1
        sleep(2)
except KeyboardInterrupt:
    print("\n end")

