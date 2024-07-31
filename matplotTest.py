import matplotlib.pyplot as plt
from kinematics import kinematics
#from ipywidgets import *
from math import *


# make the plot larger
plt.rcParams['figure.dpi'] = 150



k = kinematics()

offsets = {}

# offsets = {
#     0: {0: radians(-231), 1: radians(-129), 2: radians(-21 + 90)},  # Offsets for left front leg
#     1: {0: radians(-231), 1: radians(-129 ), 2: radians(-21 + 90)}, # Offsets for left back leg
#     2: {0: radians(-125), 1: radians(-126 - 90), 2: radians(-21)},  # Offsets for right front leg
#     3: {0: radians(-106), 1: radians(-126 - 90), 2: radians(-21)}  # Offsets for right back leg
# }
def move_robot(roll =0, pitch = 0, yaw = 0, x = 0, y = 0, z = -0.145, rot_x= 0, rot_y = 0, rot_z= 0):
    servo_angles = k.get_servo_angles() 
    k.plot_robot(xyz=[[x,y,z],[x,y,z],[x,y,z],[x,y,z]], rot=[roll,pitch,yaw], 
                 is_radians=False, center_offset=[rot_x,rot_y,rot_z], offsets = offsets)
    
    


    L_Front_Angles = servo_angles[0]
    L_Back_Angles = servo_angles[1]
    R_Front_Angles = servo_angles[2]
    R_Back_Angles = servo_angles[3]


        

    print("L Front Angles", L_Front_Angles)
    print("L Back angles: ", L_Back_Angles)
    print("R Front Angles: ", R_Front_Angles)
    print("R Back Angles: ", R_Back_Angles)

    
move_robot(z = -0.20)





