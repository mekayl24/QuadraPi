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

kit.servo[L_Back_Hip].angle = 100
kit.servo[L_Back_Ankle].angle =90 #increasing == more perpendicular
kit.servo[L_Back_Knee].angle = 90

kit.servo[R_Back_Hip].angle = 102
kit.servo[R_Back_Ankle].angle = 90
kit.servo[R_Back_Knee].angle = 90


# make the plot larger
plt.rcParams['figure.dpi'] = 150

k = kinematics()


offsets = {
    0: {0: radians(-240.5), 1: radians(-130.5), 2: radians(-12.15)},  # Offsets for left front leg
    1: {0: radians(-240.5), 1: radians(-130.5), 2: radians(-12.15)}, # Offsets for left back leg
    2: {0: radians(-116.5), 1: radians(-130.5), 2: radians(-12.15)},  # Offsets for right front leg
    3: {0: radians(-97.5), 1: radians(-130.5), 2: radians(-12.15)}  # Offsets for right back leg
}



def move_robot(roll = 0, pitch = 0, yaw = 0, x = 0, y = 0, z = -0.15, rot_x = 0, rot_y = 0, rot_z = 0):
    k.plot_robot(xyz=[[x,y,z],[x,y,z],[x,y,z],[x,y,z]], rot=[roll,pitch,yaw], 
                 is_radians=False, center_offset=[rot_x,rot_y,rot_z], offsets = offsets) #Runs calculations for all IK
    
    servo_angles = k.get_servo_angles()   #retrieve angles from object


    L_Front_Angles = servo_angles[0]
    L_Back_Angles = servo_angles[1]
    R_Front_Angles = servo_angles[2]
    R_Back_Angles = servo_angles[3]       
    
    #setting all servo angles to corresponing joints

    kit.servo[L_Front_Hip].angle = L_Front_Angles[0]
    kit.servo[L_Front_Knee].angle = L_Front_Angles[1]
    kit.servo[L_Front_Ankle].angle = L_Front_Angles[2]

    time.sleep(0.01)


    kit.servo[R_Front_Hip].angle = R_Front_Angles[0]
    kit.servo[R_Front_Knee].angle = R_Front_Angles[1]
    kit.servo[R_Front_Ankle].angle = R_Front_Angles[2]

    time.sleep(0.01)


    kit.servo[L_Back_Hip].angle = L_Back_Angles[0]
    kit.servo[L_Back_Knee].angle = L_Back_Angles[1]
    kit.servo[L_Back_Ankle].angle = L_Back_Angles[2]

    time.sleep(0.01)


    kit.servo[R_Back_Hip].angle = R_Back_Angles[0]
    kit.servo[R_Back_Knee].angle = R_Back_Angles[1]
    kit.servo[R_Back_Ankle].angle = R_Back_Angles[2]

    time.sleep(0.01)



def motionTest(roll= 0, pitch =0, yaw = 0, x= 0, y= 0, z = -0.15, rot_x = 0, rot_y = 0, rot_z = 0):

    move_robot()
    move_robot(z = z)
    move_robot(z = -0.11)

    move_robot()
    move_robot(y = y)
    move_robot(y = -y)

    move_robot()
    move_robot(x =x )
    move_robot(x = -x)

    move_robot()
    move_robot(roll = roll)
    move_robot(roll = -roll)

    move_robot()
    move_robot(pitch = pitch)
    move_robot(pitch = -pitch)

    move_robot()
    move_robot(yaw = yaw)
    move_robot(yaw = -yaw)

    move_robot()

    return


#motionTest(roll = 20, pitch = 20, yaw = 20, x = 0.05, y = 0.05, z = -0.22)