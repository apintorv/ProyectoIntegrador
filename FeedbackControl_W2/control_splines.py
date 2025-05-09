import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

def spline_natural_with_derivative(x, y, num_points=50):
    n = len(x)
    h = [x[i+1] - x[i] for i in range(n - 1)]

    # Matriz tridiagonal
    A = np.zeros((n, n))
    b = np.zeros(n)
    A[0, 0] = 1
    A[-1, -1] = 1

    for i in range(1, n - 1):
        A[i, i-1] = h[i-1]
        A[i, i] = 2 * (h[i-1] + h[i])
        A[i, i+1] = h[i]
        b[i] = 6 * ((y[i+1] - y[i])/h[i] - (y[i] - y[i-1])/h[i-1])

    M = np.linalg.solve(A, b)

    x_dense = []
    y_dense = []
    dy_dense = []

    for i in range(n - 1):
        xi, xi1 = x[i], x[i+1]
        hi = h[i]
        Mi, Mi1 = M[i], M[i+1]
        yi, yi1 = y[i], y[i+1]

        ts = np.linspace(xi, xi1, num_points // (n - 1))
        for t in ts:
            a = (Mi1 * (t - xi)**3) / (6 * hi)
            b_ = (Mi * (xi1 - t)**3) / (6 * hi)
            c = (yi1/hi - Mi1 * hi / 6) * (t - xi)
            d = (yi/hi - Mi * hi / 6) * (xi1 - t)
            y_val = a + b_ + c + d

            # Derivada
            da = (Mi1 * 3 * (t - xi)**2) / (6 * hi)
            db = -(Mi * 3 * (xi1 - t)**2) / (6 * hi)
            dc = (yi1/hi - Mi1 * hi / 6)
            dd = -(yi/hi - Mi * hi / 6)
            dy_val = da + db + dc + dd

            x_dense.append(t)
            y_dense.append(y_val)
            dy_dense.append(dy_val)

    return x_dense, y_dense, dy_dense


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
        self.create_subscription(Twist, '/pose', self.position_callback, qos_profile)
        self.create_subscription(Float32MultiArray, '/path_array', self.desired_position_callback, qos_profile)

        self.qd_list = []
        self.current_target_index = 0
        self.qd = None
        self.qd_dot = None
        self.q0 = np.array([[0.0, 0.0]]).T
        self.thetha = 0.0

        self.k = 0.1
        self.h = 0.05
        self.threshold = 0.1

        self.timer = self.create_timer(0.01, self.timer_callback)

    def position_callback(self, msg):
        self.q0 = np.array([[msg.linear.x, msg.linear.y]]).T
        self.thetha = msg.angular.z

    def desired_position_callback(self, msg):
        coords = msg.data
        points = [(coords[i], coords[i+1]) for i in range(0, len(coords), 2)]
        if len(points) < 2:
            self.get_logger().warn("⚠️ Se necesitan al menos 2 puntos para interpolar.")
            return

        x = [p[0] for p in points]
        y = [p[1] for p in points]
        t = list(range(len(points)))  # parámetro artificial

        _, x_interp, dx_interp = spline_natural_with_derivative(t, x)
        _, y_interp, dy_interp = spline_natural_with_derivative(t, y)

        interpolated_points = [
            (np.array([[xi, yi]]).T, np.array([[dxi, dyi]]).T)
            for xi, yi, dxi, dyi in zip(x_interp, y_interp, dx_interp, dy_interp)
        ]

        self.qd_list = interpolated_points
        self.current_target_index = 0
        self.qd, self.qd_dot = self.qd_list[0]

        self.get_logger().info(f'🛣️ Trayectoria interpolada con {len(self.qd_list)} puntos.')

    def timer_callback(self):
        if not self.qd_list or self.current_target_index >= len(self.qd_list):
            return

        if self.qd is None or self.qd_list:
            return

        e = self.q0 - self.qd
        dist_to_target = np.linalg.norm(e)

        if dist_to_target < self.threshold:
            self.get_logger().info(f'✅ Reached point {self.current_target_index + 1}/{len(self.qd_list)}')
            self.current_target_index += 1
            if self.current_target_index < len(self.qd_list):
                self.qd, self.qd_dot = self.qd_list[self.current_target_index]
                self.get_logger().info(f'➡️ Next target: {self.qd.T}')
            else:
                self.get_logger().info('🏁 All waypoints reached. Stopping robot.')
                self.qd = None
                self.publisher.publish(Twist())
                return

        # Control: feedforward + feedback
        matrix_D = np.array([
            [np.cos(self.thetha), -self.h * np.sin(self.thetha)],
            [np.sin(self.thetha),  self.h * np.cos(self.thetha)]
        ])

        u_ff = self.qd_dot
        u_fb = -self.k * e
        u_total = u_ff + u_fb

        if np.linalg.det(matrix_D) != 0:
            U = np.linalg.inv(matrix_D) @ u_total
        else:
            U = np.array([[0.0], [0.0]])

        self.twist.linear.x = float(U[0])
        self.twist.angular.z = float(U[1])
        self.publisher.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    control = Control_Trajectory()
    rclpy.spin(control)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
