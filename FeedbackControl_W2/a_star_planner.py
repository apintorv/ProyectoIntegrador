import rclpy 
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Float32MultiArray
import numpy as np
import heapq
#import random
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import hashlib

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('a_star_planner')
        
        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth = 1
        )
        
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Vector3, '/qd', self.desired_position_callback, qos_profile)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.pose_array_pub = self.create_publisher(Float32MultiArray, '/path_array', 10)
        self.qd = None
        
        self.map = None
        self.map_info = None
        self.last_map_hash = None
        self.current_start = None
        self.current_goal = None
        
    def map_callback(self, msg):
        new_map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        new_hash = hashlib.md5(new_map_data).hexdigest()

        if new_hash == self.last_map_hash:
            return  # No cambios → no replanear

        self.map_info = msg.info
        self.map = new_map_data
        self.last_map_hash = new_hash

        self.get_logger().info('🗺️ Map updated. Replanning...')
        self.plan_path()
        
    def world_to_map(self, x, y):
        cx = self.map_info.width // 2
        cy = self.map_info.height // 2
        mx = int(round(x / self.map_info.resolution + cx))
        my = int(round(y / self.map_info.resolution + cy))
        return mx, my

        
    def desired_position_callback(self, msg):
        #self.get_logger().info(f'Desired Position: x:{msg.x}, y:{msg.y}')
        x_world = msg.x
        y_world = msg.y
        mx, my = self.world_to_map(x_world, y_world)
        self.qd = (mx, my)
        self.get_logger().info(f'🎯 Received desired goal: ({mx}, {my})')

        # Trigger path planning
        if self.map is not None:
            self.plan_path()

    # def get_random_free_cell(self):
    #     height, width = self.map.shape
    #     free_cells = np.argwhere((self.map >= 0) & (self.map < 50))
    #     if len(free_cells) < 2:
    #         self.get_logger().error('No enough free cells found!')
    #         return None, None
    #     start_idx, goal_idx = random.sample(range(len(free_cells)), 2)
    #     start = tuple(reversed(free_cells[start_idx]))
    #     goal = tuple(reversed(free_cells[goal_idx]))
    #     return start, goal

    def map_to_world(self, mx, my):
        # Ajuste para que el origen esté en el centro del mapa
        cx = self.map_info.width // 2
        cy = self.map_info.height // 2
        x = (mx - cx) * self.map_info.resolution
        y = (my - cy) * self.map_info.resolution
        return x, y

    def is_occupied(self, x, y):
        val = self.map[y, x]
        return val > 50  # Zona ocupada

    def plan_path(self):
        
        if self.qd is None:
            self.get_logger().warn('⚠️ Desired goal not yet received!')
            return

        goal = self.qd
        start = (0, 0)

        if not start or not goal:
            self.get_logger().warn('🛑 No valid cells to plan path!')
            return

        self.current_start = start
        self.current_goal = goal

        # Use the a_star_algorithm function to compute the path
        path = self.a_star_algorithm(start, goal)

        if path:
            self.publish_path(path)
            self.print_path(path)  # Print the path
        else:
            self.get_logger().info("No path found")

    def a_star_algorithm(self, start, goal):
        """A* algorithm function that computes the shortest path."""
        open_list = []
        heapq.heappush(open_list, (0, start))

        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_list:
            current = heapq.heappop(open_list)[1]

            if current == goal:
                path = self.reconstruct_path(came_from, current)
                return path

            for neighbor in self.get_neighbors(current):
                if self.is_occupied(*neighbor):
                    continue

                tentative_g_score = g_score[current] + 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))

        return None  # Return None if no path is found

    def get_neighbors(self, current):
        x, y = current
        neighbors = [(x + dx, y + dy) for dx in [-1, 0, 1] for dy in [-1, 0, 1]
                    if (dx != 0 or dy != 0) and 0 <= x + dx < self.map.shape[1] and 0 <= y + dy < self.map.shape[0]]
        return neighbors

    def heuristic(self, a, b):
        ax, ay = a
        bx, by = b
        return abs(ax - bx) + abs(ay - by)

    def reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(self.current_start)
        path.reverse()
        return path

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'

        # Publish start and end points
        start_x, start_y = self.map_to_world(self.current_start[0], self.current_start[1])
        goal_x, goal_y = self.map_to_world(self.current_goal[0], self.current_goal[1])

        start_pose = PoseStamped()
        start_pose.header.frame_id = 'map'
        start_pose.pose.position.x = start_x
        start_pose.pose.position.y = start_y
        path_msg.poses.append(start_pose)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        path_msg.poses.append(goal_pose)

        # Publish the path waypoints in world coordinates
        for (x, y) in path:
            wx, wy = self.map_to_world(x, y)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

        # Print start and end coordinates
        self.get_logger().info(f'📍 Start Point: ({start_x:.2f}, {start_y:.2f})')
        self.get_logger().info(f'📍 Goal Point: ({goal_x:.2f}, {goal_y:.2f})')

        for (x, y) in path:
            wx, wy = self.map_to_world(x, y)
            self.get_logger().info(f'📍 Waypoint: ({wx:.2f}, {wy:.2f})')  # en metros


def main():
    rclpy.init()
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
