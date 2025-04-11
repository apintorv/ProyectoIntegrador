import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class qd_Node(Node):
    def __init__(self):
        super().__init__('Desired_Position_Node')
        self.get_logger().info("Start QD node")
                
        self.publisher = self.create_publisher(Vector3, "/qd", 10)
        
        self.timer = self.create_timer(0.001, self.timer_callback)
        
    def timer_callback(self):
        qd = Vector3()
        
        qd.x = 0.0
        qd.y = 0.0
        qd.z = 0.0

        self.get_logger().info(f'qd x={qd.x}, y={qd.y}, z={qd.z}')
        
        self.publisher.publish(qd)

def main(args=None):
    rclpy.init(args=args)
    control = qd_Node()
    rclpy.spin(control)
    rclpy.shutdown()

if __name__ == '__main__':
    main()