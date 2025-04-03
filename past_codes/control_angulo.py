import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time
import signal
import sys
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class Control(Node):

    def __init__(self):
        super().__init__('Control_angulo')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth = 10
        )
        self.thetae = 0
        self.wl = 0
        self.wr = 0
        self.v = 0.2
        self.w = 0.0
        self.kw = 1

        self.create_subscription(Float32, '/lineFollowerError', self.error_callback, qos_profile)
        self.create_subscription(Float32, '/VelocityEncL', self.left_wheel_callback, qos_profile)
        self.create_subscription(Float32, '/VelocityEncR', self.right_wheel_callback, qos_profile)

        self.timer = self.create_timer(0.01, self.timer_callback)

    def left_wheel_callback(self, msg):
        self.wl = msg.data

    def right_wheel_callback(self, msg):
        self.wr = msg.data

    def error_callback(self, msg):
        self.thetae = msg.data

    def timer_callback(self):
        twist = Twist()
        
        if self.thetae > 0:
            self.w = 0.06
        elif self.thetae < 0:
            self.w = -0.06

        twist.linear.x = self.v
        twist.angular.z = self.w
        self.publisher_.publish(twist)
        self.get_logger().info(f'vref {self.v}')
        self.get_logger().info(f'wref {self.w}')

def main(args=None):
    rclpy.init(args=args)
    open_loop_control = Control()
    rclpy.spin(open_loop_control)
    open_loop_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
