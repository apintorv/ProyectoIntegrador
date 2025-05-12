import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from scipy.interpolate import CubicSpline  # [ADDED]

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

        # Parámetros de control
        self.k = 0.1
        self.h = 0.05
        self.threshold = 0.1

        self.timer = self.create_timer(0.01, self.timer_callback)

    def position_callback(self, msg):
        self.q0 = np.array([[msg.linear.x, msg.linear.y]]).T
        self.thetha = msg.angular.z

    def compute_cubic_spline(self, points, resolution=0.1):  # [ADDED]
        points = np.array(points)
        x = points[:, 0]
        y = points[:, 1]
        n = len(x)

        # Parametrize by arc length
        t = np.zeros(n)
        for i in range(1, n):
            t[i] = t[i-1] + np.linalg.norm(points[i] - points[i-1])

        # Natural cubic spline
        cs_x = CubicSpline(t, x, bc_type='natural')
        cs_y = CubicSpline(t, y, bc_type='natural')

        # Sample dense points along the curve
        t_dense = np.arange(t[0], t[-1], resolution)
        x_dense = cs_x(t_dense)
        y_dense = cs_y(t_dense)

        smooth_path = [np.array([[xi, yi]]).T for xi, yi in zip(x_dense, y_dense)]
        return smooth_path

    def desired_position_callback(self, msg):
        coords = msg.data
        points = [(coords[i], coords[i+1]) for i in range(0, len(coords), 2)]
        new_qd_list = [np.array([[x, y]]).T for x, y in points]

        if not new_qd_list:
            return

        # Función para verificar si dos trayectorias son similares
        def paths_are_similar(old_list, new_list, tolerance=0.05):
            if len(old_list) != len(new_list):
                return False
            for p1, p2 in zip(old_list, new_list):
                if np.linalg.norm(p1 - p2) > tolerance:
                    return False
            return True

        # Si el nuevo path es similar al anterior, y no hemos terminado, continuar sin reiniciar
        if paths_are_similar(self.qd_list, new_qd_list) and self.qd is not None:
            self.get_logger().info("🔁 Path similar al anterior. Continuando sin reiniciar.")
            return

        # Saltar puntos demasiado cercanos a la posición actual
        min_distance = 0.05  # 5 cm
        index = 0
        for i, pt in enumerate(new_qd_list):
            dist = np.linalg.norm(self.q0 - pt)
            if dist > min_distance:
                index = i
                break
        else:
            self.get_logger().info("⚠️ Todos los puntos están demasiado cerca. No se actualiza trayectoria.")
            return  # No hay puntos válidos

        # Interpolate smooth path with cubic splines [MODIFIED]
        raw_points = [(pt[0, 0], pt[1, 0]) for pt in new_qd_list]
        self.qd_list = self.compute_cubic_spline(raw_points, resolution=0.05)  # [MODIFIED]
        self.current_target_index = 0  # Reset index to start of smooth path [MODIFIED]
        self.qd = self.qd_list[self.current_target_index]

        self.get_logger().info(f'🛣️ New path received with {len(self.qd_list)} waypoints (smoothed).')
        self.get_logger().info(f'📍 Robot at: {self.q0.T}')
        self.get_logger().info(f'➡️ Starting at index {self.current_target_index}: {self.qd.T}')

    def timer_callback(self):
        if self.qd is None or not self.qd_list:
            return

        e = self.q0 - self.qd
        dist_to_target = np.linalg.norm(e)

        self.get_logger().info(f'🎯 Target: {self.qd.T} | 📍 Current: {self.q0.T} | 📏 Dist: {dist_to_target:.3f}')

        if dist_to_target < self.threshold:
            self.get_logger().info(f'✅ Reached point {self.current_target_index + 1}/{len(self.qd_list)}')
            self.current_target_index += 1

            if self.current_target_index < len(self.qd_list):
                self.qd = self.qd_list[self.current_target_index]
                self.get_logger().info(f'➡️ Next target: {self.qd.T}')
            else:
                self.get_logger().info('🏁 All waypoints reached. Stopping robot.')
                self.qd = None
                self.publisher.publish(Twist())  # Stop robot
                return

        # Ley de control
        matrix_D = np.array([
            [np.cos(self.thetha), -self.h * np.sin(self.thetha)],
            [np.sin(self.thetha),  self.h * np.cos(self.thetha)]
        ])

        aux = -self.k * e

        if np.linalg.det(matrix_D) != 0:
            U = np.linalg.inv(matrix_D) @ aux
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