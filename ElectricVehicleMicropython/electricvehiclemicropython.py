'''
This is the initial program for electric vehicle, written in Micropython. I'll probably convert it to C at some point.

author: GoldenOrbWeaver
date: November 2025
version: Micropython something
'''

# Importing packages
from machine import Pin, PWM
from utime import sleep

# Setting up variables
k_p=1
k_i=1
encoder=