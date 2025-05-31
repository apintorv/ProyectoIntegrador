##USED TO VERIFY MAP IS IN 0,0


from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.pub = self.create_publisher(PoseStamped, '/test_pose', 10)
        self.timer = self.create_timer(1.0, self.publish_pose)

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
