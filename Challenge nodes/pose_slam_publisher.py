import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, '/pose_stamped', 10)
        self.timer = self.create_timer(0.033, self.publish_pose)

    def publish_pose(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('odom', 'base_footprint', now)
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            position = Point()
            position.x = trans.transform.translation.x
            position.y = trans.transform.translation.y
            position.z = trans.transform.translation.z
            pose_msg.pose.position = position
            pose_msg.pose.orientation = trans.transform.rotation
            self.pose_pub.publish(pose_msg)
        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')

def main():
    rclpy.init()
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()