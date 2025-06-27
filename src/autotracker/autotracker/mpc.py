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
        self.Cd = np.array([[1, 0, 0, 0, 0, 0, 0 , 0, 0],
                            [0, 0, 1, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 1, 0, 0, 0, 0]])
        
        '''input all the moment of inertia here'''
        self.Jtp = 0 #moment of inertia of the rotor
        self.Ixx = 0 #moment of inertia about x axis 
        self.Iyy = 0 #moment of inertia about y axis 
        self.Izz = 0 #momnt of inertia about z axis

        '''the cost function matrices'''
        self.S = np.array([[0, 0 ,0],
                           [0, 1, 0],
                           [0, 0, 1]])
        self.Q = np.array([[0, 0 ,0],
                           [0, 1, 0],
                           [0, 0, 1]])
        self.R = np.array([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])


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
    
    def dis_to_til(self, ad_global, bd_global):
        ad_till_global = [np.zeros(9, 9) for _ in range(self.horizon-1)]
        bd_till_global = [np.zeros((9, 3)) for _ in range(self.horizon-1)]
        for i in range(0, self.horizon-1):
            ad_till_global[i] = np.block([[ad_global[i], bd_global[i]], [np.zeros((3, 6)), np.eye(3)]])
            bd_till_global[i] = np.block([[bd_global[i]], [np.eye(3)]])
        
        return ad_till_global, bd_till_global
        

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

        Adt, Bdt = self.dis_to_til(self.Ad_global, self.Bd_global) # 3x9 matrix for Adt and 3x3 matrix for Bdt

        Cdt = np.block([self.Cd , np.zeros((3, 3))]) # 3x9 matrix

        cDdash = np.block([[Bdt , np.zeros((9,3)) , np.zeros((9,3)), np.zeros((9,3))], 
                           [Adt[0]@Bdt, Bdt, np.zeros((9,3)), np.zeros((9,3))],
                           [Adt[1]@Adt[0]@Bdt, Adt[1]@Bdt, Bdt, np.zeros((9,3))],
                           [Adt[2]@Adt[1]@Adt[0]@Bdt, Adt[2]@Adt[1]@Bdt, Adt[2]@Bdt, Bdt]]) # 36x12 matrix
        
        aDhat = np.block([[Adt[0]],
                           [Adt[1]@Adt[0]],
                           [Adt[2]@Adt[1]@Adt[0]]
                           [Adt[3]@Adt[2]@Adt[1]@Adt[0]]])  # 36x9 matrix
        
        qDdash = np.block([[Cdt.T@self.Q@Cdt, np.zeros((9,9)), np.zeros((9,9)), np.zeros((9,9))],
                          [np.zeros((9,9)), Cdt.T@self.Q@Cdt, np.zeros((9,9)), np.zeros((9,9))],
                          [np.zeros((9,9)), np.zeros((9,9)), Cdt.T@self.Q@Cdt, np.zeros((9,9))],
                          [np.zeros((9,9)), np.zeros((9,9)), np.zeros((9,9)), Cdt.T@self.S@Cdt]])  #36x36 matrix 
        
        rDdash = np.block([[self.R, np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))],
                          [np.zeros((3,3)),self.R, np.zeros((3,3)), np.zeros((3,3))],
                          [np.zeros((3,3)), np.zeros((3,3)), self.R, np.zeros((3,3))],
                          [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), self.R]])   # 12x12 matrix 
                          
        tDdash = np.block([[self.Q@Cdt, np.zeros((3,9)), np.zeros((3,9)), np.zeros((3,9))],
                          [np.zeros((3,9)), self.Q@Cdt, np.zeros((3,9)), np.zeros((3,9))],
                          [np.zeros((3,9)), np.zeros((3,9)), self.Q@Cdt, np.zeros((3,9))],
                          [np.zeros((3,9)), np.zeros((3,9)), np.zeros((3,9)), self.S@Cdt]]) # 12x36 matrix
        
        hDhat = cDdash.T@ qDdash @ cDdash + rDdash #12x12 matrix
        fDhatTranspose = np.block([[aDhat.T@qDdash@cDdash],
                          [-tDdash@cDdash]])  # 12x1 matrix
        
        del_ug_global = np.linalg.inv(hDhat) @ fDhatTranspose.T @ np.array([[x_k.T], 
                                                                            [r_g.T]])   # 12x1 matrix
        
        U_g_next = del_ug_global + self.U_g_kminus1


        return U_g_next[0] , U_g_next # 3x1 matrix for the next control input, and the 12x1 matrix for the next control input
        




        


            



            
            
        



