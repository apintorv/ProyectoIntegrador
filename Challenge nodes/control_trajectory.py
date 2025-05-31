import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
from tf_transformations import euler_from_quaternion

class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')
        self.get_logger().info("🕹️ Controller node started.")

        self.pose_sub = self.create_subscription(PoseStamped, '/pose_stamped', self.pose_callback, 10)
        self.spline_sub = self.create_subscription(Float32MultiArray, '/spline_path_points', self.spline_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.q0 = np.array([[0.0], [0.0]])
        self.thetha = 0.0
        self.qd_list = []
        self.current_target_index = 0
        self.start_time = None
        self.total_traj_time = 30.0
        self.timer_period = 0.01
        self.k = 0.1
        self.h = 0.05

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def pose_callback(self, msg):
        self.q0 = np.array([[msg.pose.position.x], [msg.pose.position.y]])
        orientation_q = msg.pose.orientation
        quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        _, _, yaw = euler_from_quaternion(quaternion)
        self.thetha = (yaw + np.pi) % (2 * np.pi) - np.pi

    def spline_callback(self, msg):
        data = msg.data
        if len(data) < 2:
            return

        self.qd_list = [np.array([[data[i]], [data[i + 1]]]) for i in range(0, len(data), 2)]
        self.start_time = self.get_clock().now()
        self.current_target_index = 0

        self.get_logger().info(f"✅ Received new spline with {len(self.qd_list)} points.")

    def timer_callback(self):
        if not self.qd_list or self.start_time is None:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        fraction = elapsed / self.total_traj_time
        index = int(fraction * len(self.qd_list))

        if index >= len(self.qd_list):
            self.cmd_pub.publish(Twist())
            self.qd_list = []
            self.get_logger().info("🏁 Trajectory finished.")
            return

        self.qd = self.qd_list[index]
        e = self.q0 - self.qd

        D = np.array([
            [np.cos(self.thetha), -self.h * np.sin(self.thetha)],
            [np.sin(self.thetha),  self.h * np.cos(self.thetha)]
        ])

        U = np.array([[0.0], [0.0]])
        if np.linalg.det(D) != 0:
            U = np.linalg.inv(D) @ (-self.k * e)

        twist = Twist()
        v = float(U[0])
        w = float(U[1])
        twist.linear.x = np.sign(v) * max(abs(v), 0.1)
        twist.angular.z = np.sign(w) * max(abs(w), 0.1)
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
