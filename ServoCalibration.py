import time
import board
import busio
from adafruit_servokit import ServoKit
import numpy as np
from numpy.linalg import inv, norm
from numpy import array, asarray, matrix
from math import *
import matplotlib.pyplot as plt
from kinematics import kinematics



kit = ServoKit(channels=16)

# Define servo channels (adjust if necessary)
L_Front_Hip = 0
L_Front_Knee = 1
L_Front_Ankle = 2

R_Front_Hip = 3
R_Front_Knee = 4
R_Front_Ankle = 5

L_Back_Hip = 6
L_Back_Ankle = 7
L_Back_Knee = 8

R_Back_Hip = 9
R_Back_Ankle = 10
R_Back_Knee = 11



# Initialize servos to 90 degrees (middle position)
kit.servo[L_Front_Hip].angle = 100
kit.servo[L_Front_Knee].angle =90
kit.servo[L_Front_Ankle].angle = 90

kit.servo[R_Front_Hip].angle = 83
kit.servo[R_Front_Knee].angle = 90 #increasing makes less perpenic
kit.servo[R_Front_Ankle].angle = 90 #increasing makes less perp.

kit.servo[L_Back_Hip].angle = 90
kit.servo[L_Back_Ankle].angle = 150 #knee perp to ground facing down, set to 180 for assembly
kit.servo[L_Back_Knee].angle = 180

kit.servo[R_Back_Hip].angle = 102
kit.servo[R_Back_Ankle].angle = 90
kit.servo[R_Back_Knee].angle = 90
