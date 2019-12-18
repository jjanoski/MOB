#!/usr/bin/python3
import smbus
import serial
from time import sleep


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

            gyroscope_x = gyro_x/131.0
            gyroscope_y = gyro_y/131.0
            gyroscope_z = gyro_z/131.0
            return [accel_x, accel_y, accel_z, gyroscope_x, gyroscope_y, gyroscope_z]
        except Exception as e:
            print("ERROR, ", e)


class pid:
    def __init__(self, left_motor, right_motor, gyroscope, serial_connectoin):
        # Gyroscope
        self.Ax, self.Ay, self.Az, self.Gx, self.Gy, self.Gz = gyroscope
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.position = 1500

        self.serial = serial_connectoin

        # Gyroscope set point 0
        self.setpoint = 0

        # Assumed time
        self.PID_time = 0.02

        # PID values
        self.Kp = 1
        self.Ki = 1
        self.Kd = 1
        # result
        self.result = 0

        # Initial integral
        self.integral = 0
        self.previous_error = 0

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def PID(self):

        # PID for angle control
        error = self.setpoint - self.Ax  # error = target - Actual

        # Get integral
        self.integral = self.integral + (error * self.PID_time)
        # Get derative
        derivative = (error - self.previous_error) / self.PID_time
        self.result = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

    def initial_position(self):
        self.serial.write(self.right_motor+' P'+str(self.position)+' T1000 \r')

    def execute(self):
        self.initial_position()
        self.PID()


class Robot:
    def __init__(self):
        # head
        self.cam  = '#15'
        self.head = '#14'

        # right arm
        self.right_shoulder = '#29'
        self.right_elbow    = '#30'
        self.right_hand     = '#31'

        # left arm
        self.left_shoulder = '#28'
        self.left_elbow    = '#27'
        self.left_hand     = '#26'

        # right leg
        self.right_hip   = '#16'
        self.right_thigh = '#17'
        self.right_knee  = '#18'
        self.right_ankle = '#19'
        self.right_foot  = '#20'

        # left leg
        self.left_hip   = '#21'
        self.left_thigh = '#22'
        self.left_knee  = '#23'
        self.left_ankle = '#24'
        self.left_foot  = '#25'

        self.right_leg = [self.right_hip, self.right_thigh, self.right_knee, self.right_ankle, self.right_foot]
        self.left_leg = [self.left_hip, self.left_thigh, self.left_knee, self.left_ankle, self.left_foot]
        self.hips = [self.right_hip, self.left_hip]

    def hip_motors(self):
        return self.hips


class Main:
    def __init__(self):
        self.sp = serial.Serial('/dev/ttyUSB0', 9600)
        self.gyro_results = []
        self.right_hip = None
        self.left_hip = None
        self.run()

    def run(self):
        self.right_hip, self.left_hip = Robot.hip_motors
        while True:
            self.gyro_results = Gyroscope()
            pid(self.right_hip, self.left_hip, self.gyro_results, self.sp)
            sleep(2)
