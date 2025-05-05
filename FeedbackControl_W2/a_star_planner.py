import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
import heapq
import random
import hashlib

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('a_star_planner')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.pose_array_pub = self.create_publisher(Float32MultiArray, '/path_array', 10)

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

    def get_random_free_cell(self):
        height, width = self.map.shape
        free_cells = np.argwhere((self.map >= 0) & (self.map < 50))
        if len(free_cells) < 2:
            self.get_logger().error('No enough free cells found!')
            return None, None
        start_idx, goal_idx = random.sample(range(len(free_cells)), 2)
        start = tuple(reversed(free_cells[start_idx]))
        goal = tuple(reversed(free_cells[goal_idx]))
        return start, goal

    def map_to_world(self, mx, my):
        x = mx * self.map_info.resolution + self.map_info.origin.position.x
        y = my * self.map_info.resolution + self.map_info.origin.position.y
        return x, y

    def is_occupied(self, x, y):
        val = self.map[y, x]
        return val >= 50 or val == -1

    def a_star(self, start, goal):
        height, width = self.map.shape

        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        def get_neighbors(pos):
            x, y = pos
            moves = [(-1,0),(1,0),(0,-1),(0,1)]
            return [(x+dx, y+dy) for dx, dy in moves if 0 <= x+dx < width and 0 <= y+dy < height]

        open_set = []
        heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, current_cost, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for neighbor in get_neighbors(current):
                x, y = neighbor
                if self.is_occupied(x, y):
                    continue
                tentative_g = current_cost + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, tentative_g, neighbor))
                    came_from[neighbor] = current
        self.get_logger().warn('No path found')
        return None

    def print_path_to_terminal(self, path, start, goal):
        print(f"\n📌 Start: {start}, Goal: {goal}")
        print("🧭 Path (grid coordinates):")
        for p in path:
            print(f"→ {p}")

        visual = np.full(self.map.shape, '.', dtype=str)
        for y in range(self.map.shape[0]):
            for x in range(self.map.shape[1]):
                if self.is_occupied(x, y):
                    visual[y, x] = '#'

        for x, y in path:
            visual[y, x] = '*'
        sx, sy = start
        gx, gy = goal
        visual[sy, sx] = 'S'
        visual[gy, gx] = 'G'

        print("\n🗺️ Grid Map View (S=start, G=goal, *=path, #=obstacle):")
        for row in visual[::-1]:
            print(''.join(row))

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'

        self.path_vectors = []
        for p in path:
            self.path_vectors.append(p)

        flattened_data = [float(item) for sublist in self.path_vectors for item in sublist]
        msg = Float32MultiArray()
        msg.data = flattened_data  
        self.pose_array_pub.publish(msg)

        for x, y in path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            wx, wy = self.map_to_world(x, y)
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'✅ Path published with {len(path)} points.')

    def plan_path(self):
        if self.map is None:
            return
        if self.current_start is None or self.current_goal is None:
            start, goal = self.get_random_free_cell()
            self.current_start = start
            self.current_goal = goal

        if self.is_occupied(*self.current_start) or self.is_occupied(*self.current_goal):
            self.get_logger().warn("⚠️ Start or goal in obstacle. Regenerating...")
            self.current_start, self.current_goal = self.get_random_free_cell()

        path = self.a_star(self.current_start, self.current_goal)
        if path:
            self.print_path_to_terminal(path, self.current_start, self.current_goal)
            self.publish_path(path)

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
