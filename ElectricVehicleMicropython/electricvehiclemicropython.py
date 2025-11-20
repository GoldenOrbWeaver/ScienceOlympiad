'''
This is the initial program for electric vehicle, written in Micropython. I'll probably convert it to C at some point.

author: GoldenOrbWeaver
date: November 2025
version: Micropython
'''

# Importing packages
from machine import Pin, PWM
from utime import sleep
from math import asin

# Setting up pins
motor_encoder=Pin(0, Pin.IN)
motor_pwm=PWM(Pin(28, Pin.OUT))
motor_pwm.freq(20000)
motor_direction=Pin(27, Pin.OUT)
ir_encoder=Pin(16, Pin.IN)


# Run variables (get these to run after inputs but before start)
run_time=10
nominal_distance=700
radius=(80**2+nominal_distance**2/4)/(2*80)
run_distance=2*radius*asin(nominal_distance/(2*radius))
time_a=0.5*(run_time-((0.5*run_time**2-4*run_distance)**0.5)/(0.8**0.5))

# Encoder loop
class Encoder:
    def __init__(self):
        self.old_distance=0
        self.distance=0 # Distance in cm
        self.speed=0 # Speed in cm/s
        self.old_value=0
        self.i=0

    def update(self):
        if self.old_value!=motor_encoder.value():
            self.distance+=0.221656815
        i+=1
        #ADD STUFF FOR SPEED
    
    def getDistance(self):
        return(self.distance)
    
    def getSpeed(self):
        return(self.speed)
    
# PI loop
class Controller():
    def __init__(self, encoder):
        self.k_p=1
        self.k_i=1
        self.error=0
        self.error_sum=0
        self.encoder=encoder

    def compute(self):
        self.error+=self.encoder.getDistance()-run_time