import numpy as np
from numpy.linalg import inv, norm
from numpy import array, asarray, matrix
from math import *
import matplotlib.pyplot as plt
from util import RotMatrix3D, point_to_rad

class kinematics():
    
    def __init__(self):
        left_front = 0
        left_back  = 1
        right_front= 2
        right_back = 3
        
        self.right_legs = [right_front, right_back]
        
        self.link_1 = 0.050
        self.link_2 = 0.115
        self.link_3 = 0.110
        self.phi = radians(90)
        
        self.length = 0.223910
        self.width = 0.205076
        self.hight = 0.0
        
        self.leg_origins = np.matrix([[self.length/2, self.width/2, 0],
                          [-self.length/2, self.width/2, 0],
                          [-self.length/2, -self.width/2, 0],
                          [self.length/2, -self.width/2, 0],
                          [self.length/2, self.width/2, 0]])
        
        self.servo_angles = {}
        
    def leg_IK(self, xyz, rot = [0,0,0], legID=0, is_radians=True, center_offset=[0,0,0], offsets = None):
        is_right = (legID in self.right_legs)
        
        XYZ = asarray((inv(RotMatrix3D(rot,is_radians)) * \
            ((array(xyz) + self.leg_origins[legID,:] - array(center_offset)).transpose())).transpose())
        
        xyz_ = asarray(XYZ - self.leg_origins[legID,:] + array(center_offset)).flatten()
        return self.leg_IK_calc(xyz_, is_right, legID, offsets)

    def leg_IK_calc(self, xyz, is_right=False, legID = 0, offsets=None): 
        x, y, z = xyz[0], xyz[1], xyz[2]
        
        len_A = norm([0,y,z])   
        a_1 = point_to_rad(y,z)                     
        a_2 = asin(sin(self.phi)*self.link_1/len_A) 
        a_3 = pi - a_2 - self.phi                   
        
        if is_right: theta_1 = a_1 - a_3
        else: 
            theta_1 = a_1 + a_3
            if theta_1 >= 2*pi: theta_1 -= 2*pi
        
        j2 = array([0,self.link_1*cos(theta_1),self.link_1*sin(theta_1)])
        j4 = array(xyz)
        j4_2_vec = j4 - j2
        
        if is_right: R = theta_1 - self.phi - pi/2
        else: R = theta_1 + self.phi - pi/2
        
        rot_mtx = RotMatrix3D([-R,0,0],is_radians=True)
        j4_2_vec_ = rot_mtx * (np.reshape(j4_2_vec,[3,1]))
        x_, y_, z_ = j4_2_vec_[0], j4_2_vec_[1], j4_2_vec_[2]
        
        len_B = norm([x_, z_])
        
        if len_B >= (self.link_2 + self.link_3): 
            len_B = (self.link_2 + self.link_3) * 0.99999
            print('target coordinate: [%f %f %f] too far away' % (x, y, z))
        
        b_1 = point_to_rad(x_, z_)  
        b_2 = acos((self.link_2**2 + len_B**2 - self.link_3**2) / (2 * self.link_2 * len_B)) 
        b_3 = acos((self.link_2**2 + self.link_3**2 - len_B**2) / (2 * self.link_2 * self.link_3))  
        
        theta_2 = b_1 - b_2    
        theta_3 = pi - b_3
        
        j1 = np.array([0,0,0])
        j3_ = np.reshape(np.array([self.link_2*cos(theta_2),0, self.link_2*sin(theta_2)]),[3,1])
        j3 = np.asarray(j2 + np.reshape(np.linalg.inv(rot_mtx)*j3_, [1,3])).flatten()
        j4_ = j3_ + np.reshape(np.array([self.link_3*cos(theta_2+theta_3),0, self.link_3*sin(theta_2+theta_3)]), [3,1])
        j4 = np.asarray(j2 + np.reshape(np.linalg.inv(rot_mtx)*j4_, [1,3])).flatten()
        
        angles = self.angle_corrector(angles=[theta_1, theta_2, theta_3], is_right=is_right, legID = legID, offsets = offsets)
        

        self.servo_angles[legID] = [degrees(angles[0]), degrees(angles[1]), degrees(angles[2])]
        #print(angles[0], angles[1], angles[2])
        
        return [angles[0], angles[1], angles[2], j1, j2, j3, j4]


    def angle_corrector(self, angles=[0,0,0], is_right=True, legID = 0, offsets = None):
        theta_1, theta_2, theta_3 = angles

        if offsets and legID in offsets:
            if 0 in offsets[legID]:
                theta_1 += offsets[legID][0]
            if 1 in offsets[legID]:
                theta_2 += offsets[legID][1]
            if 2 in offsets[legID]:
                theta_3 += offsets[legID][2]



        return [theta_1, theta_2, theta_3]
        
    def base_pose(self, rot=[0,0,0], is_radians=True, center_offset=[0,0,0]):
        offset = RotMatrix3D(rot, is_radians) * \
            (matrix(center_offset).transpose()) - matrix(center_offset).transpose()
        rotated_base = RotMatrix3D(rot, is_radians) * self.leg_origins.transpose() - offset
        return rotated_base.transpose()
    
    def leg_pose(self, xyz, rot, legID, is_radians, center_offset=[0,0,0], offsets = None):
        pose_relative = self.leg_IK(xyz, rot, legID, is_radians, center_offset, offsets)[3:]
        pose_true = RotMatrix3D(rot,is_radians) * (array(pose_relative).transpose())
        return pose_true.transpose()
    
    def plot_base(self, ax, rot=[0,0,0], is_radians=True, center_offset=[0,0,0]):
        p = (self.base_pose(rot, is_radians, center_offset)).transpose()     
        ax.plot3D(asarray(p[0,:]).flatten(), asarray(p[1,:]).flatten(), asarray(p[2,:]).flatten(), 'r')
        return
       
    def plot_leg(self, ax, xyz, rot=[0,0,0], legID=0, is_radians=True, center_offset=[0,0,0], offsets = None):
        p = ((self.leg_pose(xyz, rot, legID, is_radians, center_offset, offsets) \
                + self.base_pose(rot,is_radians,center_offset)[legID]).transpose())
        ax.plot3D(asarray(p[0,:]).flatten(), asarray(p[1,:]).flatten(), asarray(p[2,:]).flatten(), 'b')
        
        # Retrieve the angles for the leg
        angles = (self.leg_IK(xyz, rot, legID, is_radians, center_offset, offsets)[:3])
        
        angle1 = degrees(angles[0])
        angle2 = degrees(angles[1])
        angle3 = degrees(angles[2])
        
        #print(f"Leg ID: {legID}, Servo Angles: {angle1, angle2, angle3}")
        
        return

    def plot_robot(self, xyz, rot=[0,0,0], leg_N=4, is_radians=True, limit=0.250, center_offset=[0,0,0], offsets = None):
        ax = self.ax_view(limit)
        self.plot_base(ax,rot, is_radians, center_offset)
        for leg in range(leg_N):
            self.plot_leg(ax,xyz[leg],rot,leg, is_radians, center_offset, offsets) 
        plt.show()
        return 
    
    def get_servo_angles(self):
        return self.servo_angles
        
        
        
        
#         angles[1] -= 1.5*pi
#         if is_right:
#             theta_1 = angles[0] - pi
#             theta_2 = angles[1] + 45*pi/180
#         else: 
#             if angles[0] > pi:  
#                 theta_1 = angles[0] - 2*pi
#             else: theta_1 = angles[0]
#             theta_2 = -angles[1] - 45*pi/180
#         theta_3 = -angles[2] + 45*pi/180
        return [theta_1, theta_2, theta_3]
    
    @staticmethod
    def ax_view(limit):
        ax = plt.axes(projection="3d")
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_zlim(-limit, limit)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        return ax