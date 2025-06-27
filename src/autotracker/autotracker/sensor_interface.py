import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry

class SensorInterface:
    def __init__(self, node):
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.node = node
        self.odometry = None
        node.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos)

    def odom_cb(self, msg):
        self.odometry = msg

    def quaternion_to_euler(qx, qy, qz, qw):
        """
        Converts quaternion (qx, qy, qz, qw) to Euler angles (roll φ, pitch θ, yaw ψ).
        Returns angles in radians.
        """

        # Roll (φ)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx ** 2 + qy ** 2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (θ)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = np.pi / 2 * np.sign(sinp)  # use 90° if out of domain
        else:
            pitch = np.arcsin(sinp)

        # Yaw (ψ)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy ** 2 + qz ** 2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([[roll], [pitch], [yaw]])  # φ, θ, ψ  

    def get_state(self):
        return self.odometry

    def get_quaternion(self):
        return self.odometry.q

    def get_euler(self):
        qx, qy, qz, qw = self.odometry.q
        return quaternion_to_euler(qx, qy, qz, qw)
    
    def get_position(self):
        return np.array([self.odometry.position[0], 
                         self.odometry.position[1], 
                         self.odometry.position[2]])
    def get_velocity(self):
        return np.array([self.odometry.velocity[0], 
                         self.odometry.velocity[1] 
                         self.odometry.velocity[2]])
    def get_angular_velocity(self):
        return np.array([self.odometry.angular_velocity[0], 
                         self.odometry.angular_velocity[1], 
                         self.odometry.angular_velocity[2]])S
