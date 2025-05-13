import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from scipy.interpolate import CubicSpline

class Control_Trajectory(Node):
    def __init__(self):
        super().__init__('Control_trajectory')
        self.get_logger().info("🚀 Control node started.")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.twist = Twist()
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_subscription(Twist, '/pose_kalman', self.position_callback, qos_profile)
        self.create_subscription(Float32MultiArray, '/path_array', self.desired_position_callback, qos_profile)

        self.qd_list = []
        self.current_target_index = 0
        self.qd = None
        self.q0 = np.array([[0.0, 0.0]]).T
        self.thetha = 0.0

        self.k = 0.1
        self.h = 0.05
        self.total_traj_time = 10.0  # Duration to traverse trajectory in seconds
        self.start_time = None

        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def position_callback(self, msg):
        self.q0 = np.array([[msg.linear.x, msg.linear.y]]).T
        self.thetha = msg.angular.z

    def compute_cubic_spline(self, points):
        # Remove consecutive duplicate points (or nearly identical)
        filtered_points = [points[0]]
        for p in points[1:]:
            if np.linalg.norm(np.array(p) - np.array(filtered_points[-1])) > 1e-6:
                filtered_points.append(p)

        points = np.array(filtered_points)
        x = points[:, 0]
        y = points[:, 1]
        n = len(x)

        if n < 2:
            self.get_logger().warn("❗ Not enough unique points for spline.")
            return []

        # Generate arc-length parameter t
        t = np.zeros(n)
        for i in range(1, n):
            t[i] = t[i - 1] + np.linalg.norm(points[i] - points[i - 1])

        if np.any(np.diff(t) <= 0):
            self.get_logger().error("❌ t is not strictly increasing. Aborting spline.")
            return []

        cs_x = CubicSpline(t, x, bc_type='natural')
        cs_y = CubicSpline(t, y, bc_type='natural')

        num_points = int(self.total_traj_time / self.timer_period)
        t_dense = np.linspace(t[0], t[-1], num_points)
        x_dense = cs_x(t_dense)
        y_dense = cs_y(t_dense)

        smooth_path = [np.array([[xi, yi]]).T for xi, yi in zip(x_dense, y_dense)]
        return smooth_path

    def desired_position_callback(self, msg):
        coords = msg.data
        points = [(coords[i], coords[i + 1]) for i in range(0, len(coords), 2)]

        if not points:
            return

        # Insert current position at the start of the trajectory
        current_pos = (self.q0[0, 0], self.q0[1, 0])
        full_points = [current_pos] + points

        raw_points = [(x, y) for x, y in full_points]
        self.qd_list = self.compute_cubic_spline(raw_points)
        self.current_target_index = 0
        self.qd = self.qd_list[0]
        self.start_time = self.get_clock().now()

        self.get_logger().info(f'🛣️ New time-aligned path with {len(self.qd_list)} waypoints.')
        self.get_logger().info(f'📍 Start position: {self.q0.T}')
        self.get_logger().info(f'🕒 Total trajectory time: {self.total_traj_time}s')

    def timer_callback(self):
        if self.qd is None or not self.qd_list or self.start_time is None:
            return

        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        fraction = elapsed_time / self.total_traj_time
        index = int(fraction * len(self.qd_list))

        if index >= len(self.qd_list):
            self.get_logger().info('🏁 Trajectory completed. Stopping robot.')
            self.qd = None
            self.publisher.publish(Twist())
            return

        self.current_target_index = index
        self.qd = self.qd_list[self.current_target_index]

        e = self.q0 - self.qd
        dist_to_target = np.linalg.norm(e)

        self.get_logger().info(f'🎯 Target index: {self.current_target_index} | 📍 Current: {self.q0.T} | 📏 Dist: {dist_to_target:.3f}')

        matrix_D = np.array([
            [np.cos(self.thetha), -self.h * np.sin(self.thetha)],
            [np.sin(self.thetha),  self.h * np.cos(self.thetha)]
        ])

        aux = -self.k * e

        if np.linalg.det(matrix_D) != 0:
            U = np.linalg.inv(matrix_D) @ aux
        else:
            U = np.array([[0.0], [0.0]])

        v_min = 0.1
        w_min = 0.1

        self.twist.linear.x = float(np.sign(U[0]) * max(abs(U[0]), v_min))
        self.twist.angular.z = float(np.sign(U[1]) * max(abs(U[1]), w_min))

        self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    control = Control_Trajectory()
    rclpy.spin(control)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
