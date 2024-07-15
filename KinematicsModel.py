import numpy as np
from numpy.linalg import inv, norm
from numpy import array, asarray, matrix
from math import *
import matplotlib.pyplot as plt

def point_to_rad(p1, p2): # converts 2D cartesian points to polar angles in range 0 - 2pi
        
    if (p1 > 0 and p2 >= 0): return atan(p2/(p1))
    elif (p1 == 0 and p2 >= 0): return pi/2
    elif (p1 < 0 and p2 >= 0): return -abs(atan(p2/p1)) + pi
    elif (p1 < 0 and p2 < 0): return atan(p2/p1) + pi
    elif (p1 > 0 and p2 < 0): return -abs(atan(p2/p1)) + 2*pi
    elif (p1 == 0 and p2 < 0): return pi * 3/2
    elif (p1 == 0 and p2 == 0): return pi * 3/2 # edge case

class kinematics():
    def __init__(self):
        
        
        self.link_1 = 0.050
        self.link_2 = 0.115
        self.link_3 = 0.110
        self.phi = radians(90)
        
    def leg_IK(self, xyz):
    
        x, y, z = xyz[0], xyz[1], xyz[2]  #xyz is a list with the x y and z coords

        len_A = norm([0,y,z])
        a_1 = point_to_rad(y,z)                     
        a_2 = asin(sin(self.phi)*self.link_1/len_A) 
        a_3 = pi - a_2 - self.phi   

        theta_1 = a_1 - a_3

        j2 = array([0,self.link_1*cos(theta_1),self.link_1*sin(theta_1)])
        j4 = array(xyz)
        j4_2_vec = j4 - j2 # vector from j2 to j4

        R = theta_1 - self.phi - pi/2

        x_, y_, z_ = j4_2_vec[0], j4_2_vec[1], j4_2_vec[2]
        
        len_B = norm([x_, z_]) # norm(j4-j2)
        
        # handling mathematically invalid input, i.e., point too far away to reach
        if len_B >= (self.link_2 + self.link_3): 
            len_B = (self.link_2 + self.link_3) * 0.99999
            # self.node.get_logger().warn('target coordinate: [%f %f %f] too far away' % (x, y, z))
            print('target coordinate: [%f %f %f] too far away' % (x, y, z))


            
         # b_1 : angle between +ve x-axis and len_B (0 <= b_1 < 2pi)
        # b_2 : angle between len_B and link_2
        # b_3 : angle between link_2 and link_3
        b_1 = point_to_rad(x_, z_)  
        b_2 = acos((self.link_2**2 + len_B**2 - self.link_3**2) / (2 * self.link_2 * len_B)) 
        b_3 = acos((self.link_2**2 + self.link_3**2 - len_B**2) / (2 * self.link_2 * self.link_3))  
        
        # assuming theta_2 = 0 when the leg is pointing down (i.e., 270 degrees offset from the +ve x-axis)
        theta_2 = b_1 - b_2    
        theta_3 = pi - b_3
        
        # CALCULATE THE COORDINATES OF THE JOINTS FOR VISUALIZATION
        j1 = np.array([0,0,0])
        
        # calculate joint 3
        j3_ = np.reshape(np.array([self.link_2*cos(theta_2),0, self.link_2*sin(theta_2)]),[3,1])
        j3 = np.asarray(j2 + np.reshape(np.linalg.inv(rot_mtx)*j3_, [1,3])).flatten()
        
        # calculate joint 4
        j4_ = j3_ + np.reshape(np.array([self.link_3*cos(theta_2+theta_3),0, self.link_3*sin(theta_2+theta_3)]), [3,1])
        j4 = np.asarray(j2 + np.reshape(np.linalg.inv(rot_mtx)*j4_, [1,3])).flatten()
        

        return a_1,a_2,a_3



k = kinematics()
angles = k.leg_IK([0, 6, -7])
print("Calculated Angles (radians):", angles)


