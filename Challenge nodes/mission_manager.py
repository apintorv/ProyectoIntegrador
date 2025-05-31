import rclpy
from rclpy.node import Node
# from your_interfaces.srv import GetTwoPoses
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # self.client = self.create_client(GetTwoPoses, 'get_two_poses')
        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Esperando servicio get_two_poses...')

        self.publisher = self.create_publisher(Vector3, '/qd', 10)
        self.subscriber = self.create_subscription(Bool, '/goal_reached', self.goal_reached_callback, 10)

        self.pickup_pose = Vector3(x=1.0, y=2.0, z=0.0)
        self.delivery_pose = Vector3(x=4.0, y=5.0, z=0.0)
        
        # self.pickup_pose = None
        # self.delivery_pose = None
        self.state = 0  # 0=pickup, 1=delivery, 2=done

        # self.call_service()

    # def call_service(self):
    #     req = GetTwoPoses.Request()
    #     self.future = self.client.call_async(req)
    #     self.future.add_done_callback(self.service_response_callback)

    # def service_response_callback(self, future):
    #     try:
    #         response = future.result()
    #         self.pickup_pose = response.pickup_pose
    #         self.delivery_pose = response.delivery_pose

    #         self.get_logger().info(f"Pickup pose: {self.pickup_pose}")
    #         self.get_logger().info(f"Delivery pose: {self.delivery_pose}")

    #         self.send_goal(self.pickup_pose)
    #         self.state = 0

    #     except Exception as e:
    #         self.get_logger().error(f"Error en llamada al servicio: {e}")

    def send_goal(self, pose2d):
        msg = Vector3()
        msg.x = pose2d.x
        msg.y = pose2d.y
        msg.z = 0.0  # sin uso en este caso
        self.get_logger().info(f"Enviando goal: {msg}")
        self.publisher.publish(msg)

    def goal_reached_callback(self, msg: Bool):
        if not msg.data:
            return
        self.get_logger().info("Goal reached recibido.")

        if self.state == 0:
            self.send_goal(self.delivery_pose)
            self.state = 1
        elif self.state == 1:
            self.get_logger().info("Misión completada.")
            self.state = 2

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
