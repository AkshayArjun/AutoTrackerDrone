from build.autotracker import autotracker
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, VehicleOdometry
import numpy as np
from autotracker.sensor_interface import SensorInterface

class MPCController:
    '''Implementation of the Model Predictive Control (MPC) controller'''

    '''there are some assumtions made here : 
        i). the quadrotor tilts by small angles at a time (this is to deal with the non-linearities in the system)
        ii). the quadrotor is mass symmetric, hence inertial crossproduct is 0
        iii). since the quadroter tilt angles are small, the transform matrix from (p , q, r) -> (phi, theta, psi) can be considered to be a I(3x3) vector
        iv). moment of inertia of all rotor is constant = Jtp
    '''
    def __init__(self):
        '''Initialize MPC parameters and state variables'''
        self.dt = 0.1  # Time step for control updates
        self.horizon = 4  # Prediction horizon
        self.state_dim = 6  # State dimension (phi, phi_dot, theta, theta_dot, psi, psi_dot)^T
        self.control_dim = 3  # Control dimension (u2 , u3, u4)^T

        '''Initialize state and control variables'''
        self.Ad_global = [np.zeros((6, 6)) for _ in range(self.horizon)]
        self.Bd_global = [np.zeros((6, 3)) for _ in range(self.horizon)]
        self.state_global = [np.zeros((6, 1)) for _ in range(self.horizon)]
        self.U_g_kminus1 = [np.zeros((3, 1)) for _ in range(self.horizon)]

        '''input all the moment of inertia here'''
        self.Jtp = 0 #moment of inertia of the rotor
        self.Ixx = 0 #moment of inertia about x axis 
        self.Iyy = 0 #moment of inertia about y axis 
        self.Izz = 0 #momnt of inertia about z axis

    def lpv(self, o_net, theta_dot, phi_dot):
        
        ''' here we use the prev state values to define the parameters at every timestep'''

        o_net = o_net #this is the net angular velocity of the rotor causing the gyro effect (o1 - o2 + o3 - o4
        theta_dot = theta_dot
        phi_dot = phi_dot

        A = np.array([
            [0, 1, 0, 0, 0, 0],
            [0, 0, 0, -self.Jtp*o_net/self.Ixx, 0, (self.Iyy - self.Izz)*theta_dot/self.Ixx],
            [0, -self.Jtp*o_net/self.Iyy, 0, 0, 0, (self.Izz - self.Ixx)*phi_dot/self.Iyy],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 1],
            [0, (self.Izz - self.Ixx)*theta_dot/(2*self.Iyy), 0, (self.Izz - self.Ixx)*phi_dot/(2*self.Iyy), 0, 0]
        ])
        B = np.array([
            [0, 0, 0],
            [1/self.Ixx, 0, 0],
            [0, 0, 0],
            [0, 1/self.Iyy, 0],
            [0, 0, 0],
            [0, 0, 1/self.Izz]
        ])
        
        ''' we then discretise them '''
        Ad = np.eye(6) + A*self.dt
        Bd = B*self.dt

        return Ad, Bd
    
    def state_succession(self, A, B, U_i ,X_prev):
        X_next = A @ X_prev + B @ U_i
        return X_next

    def mpc_controller(self, U_g_kminus1, x_k, r_g):

        self.U_g_kminus1 = U_g_kminus1
        self.state_global[0] = x_k

        #function to give us the Ad, Bd matrices throughout the horizon

        for i in range(0,self.horizon-2):
            phi_dot = self.state_global[i][1,0]
            theta_dot = self.state_global[i][3,0]
            o_net = 0

            Ad, Bd = self.lpv(phi_dot, theta_dot, o_net)

            self.Ad_global[i] = Ad
            self.Bd_global[i] = Bd
            
            U_g = self.U_g_kminus1[i+1]
            X_i = self.state_global[i]

            self.state_global[i+ 1] = self.state_succession(Ad, Bd, U_g, X_i)

        '''now to calculate the matrices for the cost function, Ad_til, Bd_til
        Cd'''


        
        


            



            
            
        



