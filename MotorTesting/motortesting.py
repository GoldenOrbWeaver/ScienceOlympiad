from machine import Pin, PWM
from utime import sleep

#Setting up pins
motor_pwm=PWM(Pin(28, Pin.OUT))
motor_pwm.freq(20000)
motor_encoder=Pin(26, Pin.IN)
motor_direction=Pin(27, Pin.OUT)
motor_direction.on()

#Testing constant speed
while True:
    motor_pwm.duty_u16(65525)
    sleep(1)
    motor_pwm.duty_u16(0)
    print("working")
    sleep(1)