import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class OdoTfPublisher(Node):
    def __init__(self):
        super().__init__('odo_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Pose2D,
            '/pose_kalman',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg: Pose2D):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'      # parent frame
        t.child_frame_id = 'lidar_link'  # child frame (robot base)

        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # Convert yaw (theta) to quaternion
        qz = math.sin(msg.theta / 2.0)
        qw = math.cos(msg.theta / 2.0)

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdoTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
