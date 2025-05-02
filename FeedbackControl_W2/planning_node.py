import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Vector3
import numpy as np

from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

class Planning_Node(Node):
    def __init__(self):
        super().__init__('Planning_Node')
        self.map = None
        self.map_info = None
        self.start_pose = (0, 0)  # celda inicial (en coordenadas del grid)

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(Vector3, '/qd', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

    def map_callback(self, msg):
        self.map_info = msg.info
        data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        # Crear matriz 0 (libre) / 1 (ocupado) para pathfinding
        self.map = np.where(data > 50, 1, 0).tolist()

    def goal_callback(self, msg):
        if self.map is None:
            self.get_logger().warn('Map not received yet.')
            return

        gx = int((msg.point.x - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((msg.point.y - self.map_info.origin.position.y) / self.map_info.resolution)

        path_cells = self.find_path_with_lib(self.start_pose, (gx, gy))
        if path_cells:
            self.publish_path(path_cells)
        else:
            self.get_logger().warn('No path found.')

    def find_path_with_lib(self, start, goal):
        try:
            grid = Grid(matrix=self.map)
            start_node = grid.node(*start)
            goal_node = grid.node(*goal)

            finder = AStarFinder()
            path, _ = finder.find_path(start_node, goal_node, grid)

            return path
        except Exception as e:
            self.get_logger().error(f'Pathfinding failed: {e}')
            return None

    def publish_path(self, path_cells):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in path_cells:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x * self.map_info.resolution + self.map_info.origin.position.x
            pose.pose.position.y = y * self.map_info.resolution + self.map_info.origin.position.y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Path published with {len(path_msg.poses)} poses.')

def main(args=None):
    rclpy.init(args=args)
    node = Planning_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
