import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math
import numpy as np
import time

class talker(Node):
    def __init__(self):
        super().__init__('Talker')
        # self.declare_parameters(
        #     namespace='',
        #     parameters=[
        #         ('kpr', 1.0),
        #         ('kpt', 1.0),
        #         ('wheel_base', 0.05),
        #         ('wheel_radius', 0.19),
        #         ('positions',[[0, 0], [2, 0], [2, 2], [0, 2], [0,0]]),
        #         ('vmax',1),
        #         ('kact',100),
        #     ]
        # )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        #self.subscriber_wl = self.create_subscription(Float32, '/wl', self.callback_wl, 10)
        #self.subscriber_wr = self.create_subscription(Float32, '/wr', self.callback_wr, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Talker node successfully initialized')
        
        self.dt = 0.1  # time step (s)
        self.w_max = np.pi / 2  # maximum angular velocity (rad/s)
                
        self.x = 0.0  # x-position (m)
        self.y = 0.0  # y-position (m)
        self.theta = 0  # orientation (rad)

        self.wr = 0.0
        self.wl = 0.0
        self.i = 1
        self.error = 0.0
        
    def calculate_distance(self):
        # Get current position and orientation
        self.thetad = math.atan2((self.yd-self.y), (self.xd-self.x))
        self.error = math.sqrt((self.xd-self.x)**2 + (self.yd-self.y)**2)

        self.thetae = (self.theta - self.thetad)
        if self.thetae > math.pi:
            self.thetae = self.thetae - 2*math.pi
        elif self.thetae < -math.pi:
            self.thetae = self.thetae + 2*math.pi
            
    def timer_callback(self):
        self.kpr = self.get_parameter('kpr').get_parameter_value().double_value  
        self.kpt = self.get_parameter('kpt').get_parameter_value().double_value
        self.wheelbase = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheelradius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.vmax = self.get_parameter('vmax').get_parameter_value().integer_value
        self.kact = self.get_parameter('kact').get_parameter_value().integer_value
        self.positions = self.get_parameter('positions').get_parameter_value().array_value

        self.num_positions = self.positions.shape[0]
        
        twist = Twist()
        
        if self.i < self.num_positions:
            self.calculate_distance()
             
            self.xd = self.positions[self.i,0]
            self.yd = self.positions[self.i,1]
            
            self.W = -self.w_max*math.tanh(self.kpr*self.thetae/self.w_max)
            
            self.act = -((math.tanh*(self.kact*(abs(self.thetae)-math.pi/8)))-1)/2
            self.V = self.vmax*math.tanh*((self.kpt*self.error)/self.vmax)*self.act
        
            self.v_real = self.wheelradius* (self.wr+self.wl)/2
            self.w_real = self.wheelradius* (self.wr-self.wl)/self.wheelbase

            self.vx = self.v_real * math.cos(self.theta)
            self.vy = self.v_real * math.sin(self.theta)

            self.x = self.x + self.vx*self.dt
            self.y = self.y + self.vy*self.dt
            self.theta = self.theta + self.w_real * self.dt
  
            # Compute wheel velocities from desired linear and angular velocities
            twist.linear.x = self.V
            twist.angular.z = self.W
            self.publisher.publish(twist)

        # Check if the robot has reached the desired position
        if abs(self.error) < 0.1:
            self.i += 1
            self.calculate_distance()
        #self.check_distance()
        self.t = self.t + self.dt
        
    # def callback_wl(self,data):
    #     self.wl = data.data
        
    # def callback_wr(self,data):
    #     self.wr = data.data
        
        
def main(args=None):
    rclpy.init(args=args)
    m_p = talker()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()