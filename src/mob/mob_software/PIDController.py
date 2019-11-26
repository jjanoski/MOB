#!/usr/bin/env python

class PIDController():
    def __init__(self, leftMotor, rightMotor, gyroscope):
        # Gyroscope
        self.gyro = gyroscope
        
        #TODO: incoroprate motors
        # self.robotWalk = mob.setup.motor_driver(leftMotor, rightMotor)
        
        # Gyroscope set point 0
        self.setpoint = 0;
        
        # Assumed time
        self.PID_time = 0.02

        # PID values
        self.Kp = 1
        self.Ki = 1
        self.Kd = 1
        # result
        self.result = 0

        # Initial integral
        self.integral       = 0
        self.previous_error = 0

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def PID():
        # PID for angle control
        error = self.setpoint - self.gyro.getAngle() # error = target - Actual
        
        # Get integral
        self.integral = integral + (error*self.PID_time)
        # Get derative
        derivative = (error - self.previous_error)/self.PID_time
        self.result = self.Kp*error + self.Ki*self.integral + self.Kd*derivative
    
    def execute(self):
        self.PID()
        # TODO: incororate motors
        # self.robotWalk = mob.setup.walk(0, self.result)
