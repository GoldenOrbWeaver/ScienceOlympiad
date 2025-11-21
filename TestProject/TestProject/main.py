from machine import Pin, PWM
from utime import sleep

#Setting up pins
motor_pwm=PWM(Pin(28, Pin.OUT))
motor_pwm.freq(20000)
motor_pwm.duty_u16(0)
motor_encoder=Pin(26, Pin.IN)
motor_direction=Pin(27, Pin.OUT)
motor_direction.on()

encoder = Pin(16, Pin.IN, Pin.PULL_UP)
past_value=0
current_value=0
distance=0
i=0
old_distance=0

while(True):
    current_value=1-motor_encoder.value()
    if current_value!=past_value:
        distance+=0.7854
    #print("Distance is "+str(distance))
    if i%100==0:
        print("Speed is "+str((distance-old_distance)/0.100)+"cm/s")
        print("Distance is "+str(distance))
        old_distance=distance
        motor_pwm.duty_u16(65525*int((-1)**(i/100)))
    past_value=current_value
    i+=1
    sleep(0.001)