import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import tf_transformations
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

class RobotPoseNode(Node):
    def __init__(self):
        super().__init__('robot_pose_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, '/pose_stamped', 10)
        self.timer = self.create_timer(0.1, self.get_robot_pose)

    def get_robot_pose(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            position = Point()
            position.x = trans.transform.translation.x
            position.y = trans.transform.translation.y
            position.z = self.get_yaw_from_quaternion(trans.transform.rotation)
            pose_msg.pose.position = position
            pose_msg.pose.orientation = trans.transform.rotation
            self.pose_pub.publish(pose_msg)
            # self.get_logger().info(f'Robot: x={position.x}, '
            #                        f'y={position.y}, '
            #                        f'yaw={position.z }')
        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')

    def get_yaw_from_quaternion(self, q):
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
        return yaw

def main():
    rclpy.init()
    node = RobotPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
