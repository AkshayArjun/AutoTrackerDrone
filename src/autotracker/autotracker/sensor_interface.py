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

    def get_state(self):
        return self.odometry
