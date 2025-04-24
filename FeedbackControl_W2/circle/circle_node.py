import rclpy
from rclpy.node import Node
import numpy as np
from msgs_circle.msg import Circle
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class Circle_Node(Node):
    def __init__(self):
        super().__init__('Circle_Node')
        self.get_logger().info("Start Circle node")
        
        self.dt = 0.1
        self.r = 0.5
        self.t = 0.0
                
        self.publisher = self.create_publisher(Circle, '/qd_qddot', 1)        
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
    def timer_callback(self):
        msg = Circle()
        
        msg.x = self.r * np.cos(self.t)
        msg.y =  self.r * np.sin(self.t)
        msg.x_dot =  -self.r * np.sin(self.t)
        msg.y_dot =  self.r *np.cos(self.t)
        
        self.get_logger().info(f'qd x={msg.x}, y={msg.y}, x_dot={msg.x_dot}, y_dot={msg.y_dot}')        
        self.publisher.publish(msg)
        
        self.t += self.dt

def main(args=None):
    rclpy.init(args=args)
    circle = Circle_Node()
    rclpy.spin(circle)
    rclpy.shutdown()

if __name__ == '__main__':
    main()