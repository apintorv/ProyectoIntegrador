import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import math

class Position_Node(Node):
    def __init__(self):
        super().__init__('Position_Node')
        self.get_logger().info("Start position node")

        self.subscriber = self.create_subscription(Twist, "/vel_raw", self.callback, 1)
        self.velPublisher = self.create_publisher(Twist, "/pose", 1)
              
        self.q = np.array([[0, 0, 0]]).T  # Vector de estados q = [x,y,theta]
        
        self.h = 0.05
        
        self.pose = Twist()
        
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        x = float(self.q[0][0])
        y = float(self.q[1][0])
        theta = float(self.q[2][0])

        # Position
        self.pose.linear.x = x
        self.pose.linear.y = y
        self.pose.angular.z = theta
        self.velPublisher.publish(self.pose)

        self.get_logger().info(
            f"Pose → x: {x:.2f}, y: {y:.2f}, θ: {theta:.2f}"
        )
        
    def callback(self, msg):
        # Compute time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        v = msg.linear.x
        omega = msg.angular.z
        theta = self.q[2][0]
        theta = (theta + math.pi) % (2 * math.pi) - math.pi
        
        u = np.array([[v, omega]]).T
        gx = np.array([
            [np.cos(theta), -self.h * np.sin(theta)],
            [np.sin(theta), self.h * np.cos(theta) ],
            [          0,                          1             ]
        ])
        
        self.q = self.q + dt * (gx @ u)  
        
def main(args=None):
    rclpy.init(args=args)
    position = Position_Node()
    rclpy.spin(position)
    rclpy.shutdown()

if __name__ == '__main__':
    main()