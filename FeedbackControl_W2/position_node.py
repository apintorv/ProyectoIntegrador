import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
import numpy as np
import math

class Position_Node(Node):
    def __init__(self):
        super().__init__('Position_Node')
        self.get_logger().info("Start position node")

        ##suscribirse al nodo del robot que da los sensores 
        self.subscriber = self.create_subscription(Twist, "/cmd_vel", self.callback, 10)
        self.velPublisher = self.create_publisher(Pose, "/pose", 10)
              
        self.q = np.array([[0, 0, 0]]).T  # Vector de estados q = [x,y,theta]
        
        self.tao = 0.001
        self.h = 0.05
        
        self.pose = Pose()
        
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        #self.get_logger().info(f"q: {self.q}")
        
        self.pose.position.x = float(self.q[0][0])
        self.pose.position.y = float(self.q[1][0])
        self.pose.orientation.w = float(self.q[2][0])
        
        self.get_logger().info(f'Actual Position: x:{self.pose.position.x}, y:{self.pose.position.y}, z:{self.pose.orientation.w}')
        self.velPublisher.publish(self.pose)
        
    def callback(self, msg):
        self.u = np.array([[msg.linear.x, msg.angular.z]]).T
        self.gx = np.array([
            [np.cos(self.q[2][0]), -self.h * np.sin(self.q[2][0])],
            [np.sin(self.q[2][0]), self.h * np.cos(self.q[2][0]) ],
            [          0,                          1             ]
        ])
        
        self.q = self.q + self.tao * (self.gx @ self.u)  
        
def main(args=None):
    rclpy.init(args=args)
    position = Position_Node()
    rclpy.spin(position)
    rclpy.shutdown()

if __name__ == '__main__':
    main()