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
import time

# Setting up pins
motor_encoder=Pin(0, Pin.IN)
motor_pwm=PWM(Pin(28, Pin.OUT))
motor_pwm.freq(20000)
motor_direction=Pin(27, Pin.OUT)
ir_encoder=Pin(16, Pin.IN)


# Run variables (get these to run after inputs but before start)
run_time=10 # time in s
nominal_distance=700 # distance in cm
radius=(80**2+nominal_distance**2/4)/(2*80)
run_distance=2*radius*asin(nominal_distance/(2*radius))
time_a=0.5*(run_time-((0.5*run_time**2-4*run_distance)**0.5)/(0.8**0.5))

# Encoder loop
class Encoder:
    def __init__(self, interval: int):
        self.old_distance=0
        self.distance=0 # Distance in cm
        self.speed=0 # Speed in cm/s
        self.old_value=0
        self.i=0
        self.interval=interval

    def update(self):
        if self.old_value!=motor_encoder.value():
            self.distance+=0.221656815
        if i%100==0: #Check if 100 works or if you need to change this
            self.speed=(self.distance-self.old_distance)/(self.interval*100)
            self.old_distance=self.distance
        i+=1
    
    def getDistance(self):
        return(self.distance)
    
    def getSpeed(self):
        return(self.speed)
    
# PI loop
class Controller():
    def __init__(self, encoder: Pin, interval: int, start):
        self.k_p=1
        self.k_i=1
        self.error=0
        self.error_sum=0
        self.encoder=encoder
        self.interval=interval
        self.start=start

    def compute(self):
        time_elapsed=time.ticks_diff(time.ticks_ms(), self.start)
        if time_elapsed<=time_a*1000: # Accelerating section
            self.error=0.5*0.5*((time_elapsed/1000)**2)-self.encoder.getDistance()
        elif time_elapsed>time_a*1000 and time_elapsed<(run_time-time_a)*1000: # Constant velocity section
            self.error=0.5*0.5*time_a**2+(time_elapsed/1000-time_a)*time_a*0.5-self.encoder.getDistance()
        elif time_elapsed>=(run_time-time_a)*1000: # Decelerating section
            self.error=0.5*0.5*time_a**2+(time_elapsed/1000-time_a)*time_a*0.5-0.5*0.5*(time_a-run_time+time_elapsed/1000)**2-self.encoder.getDistance()
        else: # After time
            self.error=run_distance-self.encoder.getDistance()
        self.error_sum+=self.error