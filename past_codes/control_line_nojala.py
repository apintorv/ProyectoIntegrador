import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32
import math
import numpy as np
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class OpenLoopControl(Node):
    def __init__(self):
        super().__init__('open_loop_control')

        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth = 10
        )

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber_wl = self.create_subscription(Float32, '/VelocityEncL', self.callback_wl, qos_profile)
        self.subscriber_wr = self.create_subscription(Float32, '/VelocityEncR', self.callback_wr, qos_profile)
        self.subscriber_line = self.create_subscription(Pose2D, '/box_center', self.callback_line, qos_profile)
        self.subscriber_frame = self.create_subscription(Pose2D, '/frame_center', self.callback_frame, qos_profile)
        self.timer = self.create_timer(0.01, self.move)
        self.get_logger().info('Talker node successfully initialized')

        self.vmax = 0.8
        self.wheel_radius = 0.0515  # radius of wheels (m)
        self.wheelbase = 0.195  # distance between wheels (m) (l)
        self.dt = 0.0 # time step (s)

        self.last_time = self.get_clock().now()

        #self.positions = np.array([[0, 0], [1, 0], [1, 1], [0, 1], [0,0]])
        #self.num_positions = self.positions.shape[0]
        #self.get_logger().info(f'puntos {self.num_positions}')

        # self.x = 640.0  # x-position (m)
        # self.y = 720.0  # y-position (m)
        self.theta = 0  # orientation (rad)

        self.kpr = 0.4
        self.kpt = 0.5
        self.wr = 0.0
        self.wl = 0.0

        self.kact = 100
        self.i = 1
        self.line_error = 0.0
        self.frame_error = 0.0
        self.line_error = Pose2D()
        self.frame_error = Pose2D()

    def move(self):
        self.get_logger().info(f'vel {self.wr},{self.wr}')
        twist = Twist()

        self.xd = self.line_error.x
        self.yd = self.line_error.y
        
        self.x = self.frame_error.x
        self.y = self.frame_error.y
        
        self.get_logger().info(f'coordenadas {self.xd},{self.yd}')

        error = math.sqrt((self.xd-self.x)**2 + (self.yd-self.y)**2)
        self.get_logger().info(f'error {error}')

        # Get current position and orientation
        thetad = math.atan2((self.yd-self.y), (self.xd-self.x))
        thetae = (self.theta - thetad)
        if thetae > math.pi:
            thetae = thetae - 2*math.pi
        elif thetae < -math.pi:
            thetae = thetae + 2*math.pi

        w = -self.vmax * math.tanh(self.kpr * thetae / self.vmax)

        act = -((math.tanh(self.kact * (abs(thetae) - math.pi / 8))) - 1) / 2
        V = self.vmax * math.tanh(self.kpt * error / self.vmax) * act

        twist.angular.z = w
        twist.linear.x = V
        
        self.get_logger().info(f'w {w}')
        self.get_logger().info(f'V {V}')
        
        current_time = self.get_clock().now()
        self.dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        v_real = self.wheel_radius* (self.wr+self.wl)/2
        w_real = self.wheel_radius* (self.wr-self.wl)/self.wheelbase

        vx = v_real * math.cos(self.theta)
        vy = v_real * math.sin(self.theta)

        self.x = self.x + vx*self.dt
        self.y = self.y + vy*self.dt
        self.theta = self.theta + w_real * self.dt

        self.publisher_.publish(twist)
        
        # if abs(error) < 0.1:
        #     #vref = 0.0
        #     #V = 0.0
        #     #twist.linear.x = V
        #     #w = 0.0
        #     #twist.angular.z = w
        #     self.i += 1
        #     self.get_logger().info(f'i {self.i}')
        self.get_logger().info(f'vref {V}')
        self.get_logger().info(f'wref {w}')


    def callback_wl(self,msg):
        self.wl = msg.data

    def callback_wr(self,msg):
        self.wr = msg.data

        
    def callback_line(self,msg):
        # self.line_error = msg.data
        self.line_error.x = msg.x
        self.line_error.y = msg.y
        self.line_error.theta = msg.theta
        # self.get_logger().info(f'Line error set {self.line_error_x}')


    def callback_frame(self,msg):
            # self.line_error = msg.data
            self.frame_error.x = msg.x
            self.frame_error.y = msg.y
            # self.line_error.theta = msg.theta
            # self.get_logger().info(f'Line error set {self.line_error_x}')
        
def main(args=None):
    rclpy.init(args=args)
    open_loop_control = OpenLoopControl()
    rclpy.spin(open_loop_control)
    open_loop_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




























