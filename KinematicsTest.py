import time
import board
import busio
from adafruit_servokit import ServoKit
import numpy as np
from numpy.linalg import inv, norm
from numpy import array, asarray, matrix
from math import *
import matplotlib.pyplot as plt







kit = ServoKit(channels=16)

# Define servo channels (adjust if necessary)
L_Front_Hip = 0
L_Front_Knee = 1
L_Front_Ankle = 2

R_Front_Hip = 3
R_Front_Knee = 4
R_Front_Ankle = 5

L_Back_Hip = 6
L_Back_Knee = 7
L_Back_Ankle = 8

R_Back_Hip = 9
R_Back_Knee = 10
R_Back_Ankle = 11



# Initialize servos to 90 degrees (middle position)
kit.servo[L_Front_Hip].angle = 90
kit.servo[L_Front_Knee].angle = 90
kit.servo[L_Front_Ankle].angle = 90

kit.servo[R_Front_Hip].angle = 90
kit.servo[R_Front_Knee].angle = 100
kit.servo[R_Front_Ankle].angle = 50

kit.servo[L_Back_Hip].angle = 90
kit.servo[L_Back_Knee].angle = 90
kit.servo[L_Back_Ankle].angle = 90

kit.servo[R_Back_Hip].angle = 90
kit.servo[R_Back_Knee].angle = 90
kit.servo[R_Back_Ankle].angle = 90




def move_servo(channel, start_angle, end_angle, delay=0.01):
    step = 1 if start_angle < end_angle else -1
    for angle in range(start_angle, end_angle, step):
        kit.servo[channel].angle = angle
        time.sleep(delay)
    kit.servo[channel].angle = end_angle
""""
try:
    while True:
        # Move hip joint from 0 to 180 degrees and back
        move_servo(hip_servo_channel, 90, 180)
        move_servo(hip_servo_channel, 180, 0)
        move_servo(hip_servo_channel, 0, 90)
        
        # Move knee joint from 0 to 180 degrees and back
        move_servo(knee_servo_channel, 90, 180)
        move_servo(knee_servo_channel, 180, 0)
        move_servo(knee_servo_channel, 0, 90)

        # Move ankle joint from 0 to 180 degrees and back
        move_servo(ankle_servo_channel, 90, 180)
        move_servo(ankle_servo_channel, 180, 0)
        move_servo(ankle_servo_channel, 0, 90)

except KeyboardInterrupt:
    # Reset servos to the initial position on exit
    kit.servo[hip_servo_channel].angle = 90
    kit.servo[knee_servo_channel].angle = 90
    kit.servo[ankle_servo_channel].angle = 90
    print("Program terminated and servos reset to initial position.")
    
    """


















































