from build.autotracker import autotracker
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, VehicleOdometry
import numpy as np
from autotracker.sensor_interface import SensorInterface

class FBLController:
    '''implementation of the feedback linearisation controller'''
    def __init__(self):
        '''random constants'''
        self.g = 9.81  # gravitational acceleration in m/s^2
        self.mass = 1.0  # mass of the drone in kg

        ''' the current system states to the controller '''
        self.X = 0
        self.Y = 0
        self.Z = 0
        self.VX = 0
        self.VY = 0
        self.VZ = 0
        self.AX = 0
        self.AY = 0
        self.AZ = 0

        ''' the reference states to the controller '''
        self.X_R = 0
        self.Y_R = 0
        self.Z_R = 0
        self.VX_R = 0
        self.VY_R = 0
        self.VZ_R = 0
        self.AX_R = 0
        self.AY_R = 0
        self.AZ_R = 0

        self.psir = 0.0 #yaw angle

        '''the gains of the controller '''

        self.kx = 1.0
        self.kvx = 1.0
        
        self.ky = 1.0
        self.kvy = 1.0
        
        self.kz = 1.0
        self.kvz = 1.0

        self.weight_matrix = np.array([[self.kx],[self.ky],[self.kz],[self.kvx],[self.kvy],[self.kvz]])



    def fbl_control(self, pos_ref, vel_ref, pos_sys, vel_sys, acc_sys, acc_ref, psi_ref):

        self.X = pos_sys[0]
        self.Y = pos_sys[1]
        self.Z = pos_sys[2]

        self.VX = vel_sys[3]
        self.VY = vel_sys[4]
        self.VZ = vel_sys[5]

        self.AX = acc_sys[0]
        self.AY = acc_sys[1]
        self.AZ = acc_sys[2]

        # Set the reference position and velocity
        self.X_R = pos_ref[0]
        self.Y_R = pos_ref[1]
        self.Z_R = pos_ref[2]

        self.VX_R = vel_ref[0]
        self.VY_R = vel_ref[1]
        self.VZ_R = vel_ref[2]

        self.AX_R = acc_ref[0]
        self.AY_R = acc_ref[1]
        self.AZ_R = acc_ref[2]

        self.psir = psi_ref

        # Create column vectors using numpy for current and reference states
        x = np.array([[self.X], [self.Y], [self.Z]])
        v = np.array([[self.VX], [self.VY], [self.VZ]])
        a = np.array([[self.AX], [self.AY], [self.AZ]])

        x_r = np.array([[self.X_R], [self.Y_R], [self.Z_R]])
        v_r = np.array([[self.VX_R], [self.VY_R], [self.VZ_R]])
        a_r = np.array([[self.AX_R], [self.AY_R], [self.AZ_R]])

        sys_state = np.vstack((x,v))
        ref_state = np.vstack((x_r,v_r))

        # Calculate the error in position and velocity
        e = sys_state - ref_state

        # Calculate the control input using the weight matrix
        u = np.dot(self.weight_matrix, e)

        ddex = u[0, 0] + u[3,0]
        ddey = u[1, 0] + u[4,0]
        ddez = u[2, 0] + u[5,0]

        dde = np.array([[ddex], [ddey], [ddez]]) #error double derivative 
        ddr = np.array([[self.AX_R], [self.AY_R], [self.AZ_R]]) # reference double derivativev
        # Calculate the desired double derivatives

        dds = ddr - dde

        #calculating the outputs , theta, phi and U1
        a = dds[0, 0]/(dds[2, 0] + self.g)  # a is the acceleration in the x direction
        b = dds[1, 0]/(dds[2, 0] + self.g)  # b is the acceleration in the y direction
        c = np.cos(self.psir)
        d = np.sin(self.psir)

        theta = np.arctan2(a*c +  b*d, c**2 +  d**2)# theta is the angle in the x-y plane
        # Check if theta is between pi/4 to 3pi/4 or 5pi/4 to 7pi/4, else between 7pi/4 to pi/4 or 3pi/4 to 5pi/4
    
        if (np.pi/4 <= psi_ref <= 3*np.pi/4) or (5*np.pi/4 <= psi_ref <= 7*np.pi/4):
            phi = np.arctan2(np.cos(theta)*(a - np.tan(theta)*c), d)
        else:
            phi = np.arctan2(np.cos(theta)*(np.tan(theta)*d- a), c)

        u1 = self.mass * (dds[2, 0] + self.g)/(np.cos(phi)*np.cos(theta))  # U1 is the thrust force

        output = np.array([[u1], [phi], [theta]])
        return output
    
