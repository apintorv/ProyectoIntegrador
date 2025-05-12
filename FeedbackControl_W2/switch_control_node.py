import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import  Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class SwitchControlNode(Node):
    def __init__(self):
        super().__init__('switch_control_node')
        self.get_logger().info("Start control node")
        
        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth = 1
        )

        self.twist = Twist()
        
        self.publisher = self.create_publisher(Bool, "/decision", 1) # ver si cambiarlo por string o int
        self.create_subscription(Twist, '/theta', self.angle_callback, qos_profile)
        
        self.decision = False

        self.timer = self.create_timer(0.01, self.timer_callback)
        
    def angle_callback(self, msg):
        #self.get_logger().info(f'Actual Position: x:{msg.linear.x}, y:{msg.linear.y}, z:{msg.angular.z}')
        self.thetha = msg.angular.z   
        
    def timer_callback(self):
        
        self.publisher.publish(self.decision)

def main(args=None):
    rclpy.init(args=args)
    control = SwitchControlNode()
    rclpy.spin(control)
    rclpy.shutdown()

if __name__ == '__main__':
    main()