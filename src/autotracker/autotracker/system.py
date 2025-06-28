import rclpy
from rclpy.node import Node


from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from autotracker.sensor_interface import SensorInterface
from autotracker.trajectory import TrajectoryGenerator
from autotracker.fbl import FBLController
from autotracker.mpc import MPCController
import numpy as np

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')

        self.sensor = SensorInterface(self)
        self.trajectory = TrajectoryGenerator()
        self.fbl = FBLController()
        self.mpc = MPCController()

        self.horizon = self.mpc.horizon
        self.dt = 4*self.mpc.dt

        self.trajectory = np.zeros((3,3))

        ''' aero constants '''
        self.drag_switch = 0  # 1 for drag, 0 for no drag
        cT = 1.79e-5  # Thrust coefficient [N·s^2]
        cQ = 1.66e-7  # Torque/drage coefficient [N·m·s^2]
        l = 0.171 

        '''mpc parameters'''
        self.Ixx = self.mpc.Ixx
        self.Iyy = self.mpc.Iyy
        self.Izz = self.mpc.Izz
        self.Jtp = self.mpc.Jtp  # Moment of inertia of the rotor
        self.U_g_kminus1 = np.zeros((self.horizon*3, 1))

    import numpy as np


    def body_to_world(self,  p, q, r , qx, qy, qz, qw):
        phi, theta, psi = self.quaternion_to_euler(qx, qy, qz, qw)
        T = np.array([[1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
                      [0, np.cos(phi), -np.sin(phi)],
                      [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])
        return T @ np.array([[p], [q], [r]])

    def system_loop(self):
        ''' Initialise the system''' 

        odom = self.sensor.get_state()
        if odom is None:
            self.get_logger().info("Waiting for odometry data...")
            return
        
        pos_sys = self.sensor.get_position()
        vel_sys = self.sensor.get_velocity()

        ref_trajectory = self.trajectory.update(self.dt)
        pos_ref = ref_trajectory[0]
        vel_ref = ref_trajectory[1]
        acc_ref = ref_trajectory[2]
        psi_ref = np.arctan2(vel_ref[1], vel_ref[0])

        self.get_logger().info(f"Position: {pos_sys}, Velocity: {vel_sys}, Reference Position: {pos_ref}, Reference Velocity: {vel_ref}")

        ''' FBL Control Loop '''

        output = self.fbl.fbl_control(pos_ref, vel_ref, pos_sys, vel_sys, acc_ref, psi_ref)
        self.get_logger().info(f"Control Output, U1, phir, thetar: {output}")

        U1 = output[0]  # Thrust
        phi_ref = output[1]  # Roll angle
        theta_ref = output[2]  # Pitch angle
        r_single = np.array([[phi_ref], [theta_ref], [psi_ref]])  # Reference angles
        r_g = np.tile(r_single, (4, 1))  # Reference trajectory for MPC

        ''' MPC Control Loop '''
        for i in range(self.horizon):
            o = 0
            x_k = self.sensor.get_euler()
            self.mpc.mpc_controller(self.U_g_kminus1, x_k, r_g, o)
        # Update the control inputs
        for i in range(self.horizon):
            self.U[i*3:(i+1)*3] = np.array([self.fbl.X, self.fbl.Y, self.fbl.Z])

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    system_control = MainController()
    rclpy.spin(system_control)
    system_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)