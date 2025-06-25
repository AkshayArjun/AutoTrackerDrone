import rclpy
from rclpy.node import Node
from px4_msgs.msg import ActuatorMotors
from autotracker.sensor_interface import SensorInterface
from autotracker.trajectory import TrajectoryGenerator
from autotracker.fbl import FBLController
from autotracker.mpc import MPCController

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')
        self.sensor = SensorInterface(self)
        self.trajectory = TrajectoryGenerator()
        self.fbl = FBLController()
        self.mpc = MPCController()

        self.motor_pub = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', 10)
        self.declare_parameter('update_rate', 10.0)
        self.dt = 1.0 / self.get_parameter('update_rate').value
        self.timer = self.create_timer(self.dt, self.control_loop)

    def control_loop(self):
        odom = self.sensor.get_state()
        if odom is None:
            self.get_logger().info("Waiting for odometry...")
            return

        pos_ref, vel_ref, acc_ref = self.trajectory.update(self.dt)
        att_ref = self.fbl.compute_reference(pos_ref, vel_ref, acc_ref)
        rotor_speeds = self.mpc.compute_rotor_speeds(att_ref, odom)

        msg = ActuatorMotors()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.control = list(rotor_speeds)
        self.motor_pub.publish(msg)
        self.get_logger().info(f"ω: {rotor_speeds}")


def main(args=None):
    rclpy.init(args=args)
    node = MainController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
