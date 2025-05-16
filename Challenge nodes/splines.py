# trajectory_generator_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy.interpolate import CubicSpline

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        self.get_logger().info(" Trajectory Generator Node Started")

        self.publisher = self.create_publisher(Float32MultiArray, '/smoothed_trajectory', 1)
        self.create_subscription(Float32MultiArray, '/path_array', self.path_callback, 1)

        self.total_traj_time = 30.0
        self.timer_period = 0.01

    def compute_cubic_spline(self, points):
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

        return list(zip(x_dense, y_dense))

    def path_callback(self, msg):
        coords = msg.data
        if not coords or len(coords) < 2:
            self.get_logger().warn("⚠️ Empty or malformed path received.")
            return

        points = [(coords[i], coords[i + 1]) for i in range(0, len(coords), 2)]
        smooth_path = self.compute_cubic_spline(points)

        if not smooth_path:
            return

        flat_path = []
        for x, y in smooth_path:
            flat_path.extend([x, y])

        path_msg = Float32MultiArray(data=flat_path)
        self.publisher.publish(path_msg)
        self.get_logger().info(f"✅ Published smoothed trajectory with {len(smooth_path)} points.")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
