import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np
from tf_transformations import euler_from_quaternion

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

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
        # self.create_subscription(Twist, '/pose_kalman', self.position_callback, qos_profile)
        self.create_subscription(PoseStamped, '/pose_stamped', self.position_callback, qos_profile)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)
        
        self.create_subscription(Float32MultiArray, '/path_array', self.desired_position_callback, qos_profile)

        self.qd_list = []
        self.current_target_index = 0
        self.qd = None
        self.q0 = np.array([[0.0, 0.0]]).T
        self.thetha = 0.0

        # Parámetros de control
        self.k = 0.2
        self.h = 0.05
        self.threshold = 0.1
        
        self.goal_reached_sent = False


        self.timer = self.create_timer(0.01, self.timer_callback)

    def wrap_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def position_callback(self, msg):

        ##FOR POSE MSG
        self.q0 = np.array([[msg.pose.position.x, msg.pose.position.y]]).T 
        # Convert quaternion to yaw
        orientation_q = msg.pose.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        self.thetha = self.wrap_angle(yaw)

        # self.q0 = np.array([[msg.linear.x, msg.linear.y]]).T
        # self.thetha = msg.angular.z

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

        self.qd_list = new_qd_list
        self.current_target_index = index
        self.qd = self.qd_list[self.current_target_index]

        self.get_logger().info(f'🛣️ New path received with {len(self.qd_list)} waypoints.')
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
                ###BORRAR
                self.goal_reached_pub.publish(Bool(data=True))
                self.goal_reached_sent = True
                ###BORRAR
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
