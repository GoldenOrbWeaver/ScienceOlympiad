'''
This is the initial program for electric vehicle, written in Micropython. I'll probably convert it to C at some point.

author: GoldenOrbWeaver
date: Spring 2026
version: Micropython
'''

# Importing packages
from machine import Pin, PWM
from utime import sleep
import utime
from math import asin

# Encoder loop
class Encoder:
    def __init__(self, interval: int, motor_encoder: Pin):
        self.old_distance=0
        self.distance=0 # Distance in cm
        self.speed=0 # Speed in cm/s
        self.old_value=0
        self.i=0
        self.interval=interval #Interval in seconds
        self.motor_encoder=motor_encoder

    def update(self):
        if self.old_value!=self.motor_encoder.value():
            self.distance+=0.221656815
        if self.i%100==0: # Check if 100 works or if you need to change this
            self.speed=(self.distance-self.old_distance)/(self.interval*100)
            self.old_distance=self.distance
        self.i+=1
    
    def getDistance(self):
        return(self.distance)
    
    def getSpeed(self):
        return(self.speed)
    
# PI loop
class Controller():
    def __init__(self, encoder, interval: int, start, time_a: float, run_time: float, run_distance: float):
        self.k_p=1
        self.k_i=1
        self.error=0
        self.error_sum=0
        self.encoder=encoder
        self.interval=interval # Interval in seconds
        self.start=start
        self.time_a=time_a
        self.run_time=run_time
        self.run_distance=run_distance
        self.time_elapsed=0

    def compute(self):
        self.encoder.update()
        self.time_elapsed=utime.ticks_diff(utime.ticks_ms(), self.start)
        if self.time_elapsed<=self.time_a*1000: # Accelerating section
            self.error=0.5*0.5*((self.time_elapsed/1000)**2)-self.encoder.getDistance()
        elif self.time_elapsed>self.time_a*1000 and self.time_elapsed<(self.run_time-self.time_a)*1000: # Constant velocity section
            self.error=0.5*0.5*self.time_a**2+(self.time_elapsed/1000-self.time_a)*self.time_a*0.5-self.encoder.getDistance()
        elif self.time_elapsed>=(self.run_time-self.time_a)*1000: # Decelerating section
            self.error=0.5*0.5*self.time_a**2+(self.time_elapsed/1000-self.time_a)*self.time_a*0.5-0.5*0.5*(self.time_a-self.run_time+self.time_elapsed/1000)**2-self.encoder.getDistance()
        else: # After time
            self.error=self.run_distance-self.encoder.getDistance()
        self.error_sum+=self.error
        return self.k_p*self.error+self.k_i*self.error_sum

    def getTimeElapsed(self):
        return self.time_elapsed

class Program():
    def __init__(self):
       # Initializes program class, which controls the overall program

        # Setting up pins
        motor_encoder=Pin(0, Pin.IN)
        motor_pwm=PWM(Pin(28, Pin.OUT))
        motor_pwm.freq(20000)
        motor_direction=Pin(27, Pin.OUT)
        interval=0.001

        # Gain
        gain=1

        # Setting up data logger
        log=open("data.csv", "w")

        # Run variables (get these to run after inputs but before start)
        run_time=10 # time in s
        nominal_distance=700 # distance in cm
        radius=(80**2+nominal_distance**2/4)/(2*80)
        run_distance=2*radius*asin(nominal_distance/(2*radius))
        time_a=0.5*(run_time-((0.5*run_time**2-4*run_distance)**0.5)/(0.8**0.5)) # THIS NEEDS TO BE FIXED

        # Creating encoder
        encoder=Encoder(interval, motor_encoder)

        # Creating controller and running
        start=utime.ticks_ms()
        controller=Controller(encoder, interval, start, time_a, run_time, run_distance)
        while True:
            log.write(str(controller.getTimeElapsed()) + "," + str(encoder.getDistance()) + "\n")
            log.flush()
            motor_pwm.duty_u16(gain*controller.compute()/65535)
            if controller.getTimeElapsed/1000-run_time>=0.1:
                break
            sleep(interval)
        motor_pwm.duty_u16(0)
        log.close()

#--main section--
def main():
    program=Program()

#--execution section--
if __name__=="__main__":
    main()