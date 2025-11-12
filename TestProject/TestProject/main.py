from machine import Pin
from utime import sleep

encoder = Pin(16, Pin.IN, Pin.PULL_UP)
past_value=0
current_value=0
distance=0
i=0
old_distance=0

while(True):
    current_value=1-encoder.value()
    if current_value!=past_value:
        distance+=0.7854
    #print("Distance is "+str(distance))
    if i%100==0:
        print("Speed is "+str((distance-old_distance)/0.100)+"cm/s")
        print("Distance is "+str(distance))
        old_distance=distance
    past_value=current_value
    i+=1
    sleep(0.001)