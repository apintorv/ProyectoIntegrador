import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class qd_Node(Node):
    def __init__(self):
        super().__init__('Desired_Position_Node')
        self.get_logger().info("Start QD node")
                
        self.publisher = self.create_publisher(Vector3, "/qd", 1)
        
        # self.timer = self.create_timer(0.1, self.timer_callback)
        self.timer = self.create_timer(10, self.timer_callback)
        
        self.qd = Vector3()
        
        self.qd.x = 0.0
        self.qd.y = 0.0
        self.qd.z = 0.0
        
    def timer_callback(self):
        self.get_logger().info(f'qd x={self.qd.x}, y={self.qd.y}, z={self.qd.z}')        
        self.publisher.publish(self.qd)

def main(args=None):
    rclpy.init(args=args)
    control = qd_Node()
    rclpy.spin(control)
    rclpy.shutdown()

if __name__ == '__main__':
    main()