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
        self.subscriber = self.create_subscription(Twist, "/vel_raw", self.callback, 1)
        self.velPublisher = self.create_publisher(Pose, "/pose", 1)
              
        self.q = np.array([[0, 0, 0]]).T  # Vector de estados q = [x,y,theta]
        
        self.tao = 0.01
        self.h = 0.05
        
        self.pose = Pose()
        
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        x = float(self.q[0][0])
        y = float(self.q[1][0])
        theta = float(self.q[2][0])

        # Position
        self.pose.position.x = x
        self.pose.position.y = y
        self.pose.position.z = theta
        self.velPublisher.publish(self.pose)

        self.get_logger().info(
            f"Pose → x: {x:.2f}, y: {y:.2f}, θ: {theta:.2f} rad"
        )
        
    def callback(self, msg):
        # Compute time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        if dt <= 0:
            dt = 1e-3  # minimal safe dt

        v = msg.linear.x
        omega = msg.angular.z

        theta = self.q[2][0]
        
        u = np.array([[v, omega]]).T
        gx = np.array([
            [np.cos(theta), -self.h * np.sin(theta)],
            [np.sin(theta), self.h * np.cos(theta) ],
            [          0,                          1             ]
        ])
        
        self.q = self.q + dt * (gx @ u)  
        
        self.q[2][0] = (self.q[2][0] + math.pi) % (2 * math.pi) - math.pi
        
def main(args=None):
    rclpy.init(args=args)
    position = Position_Node()
    rclpy.spin(position)
    rclpy.shutdown()

if __name__ == '__main__':
    main()