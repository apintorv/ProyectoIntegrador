import rclpy 
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Vector3, Twist
from std_msgs.msg import Float32MultiArray
import numpy as np
import heapq
import hashlib

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('a_star_planner')
        
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Vector3, '/qd', self.desired_position_callback, 10)
        self.create_subscription(Twist, '/pose', self.position_callback, 1)

        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.pose_array_pub = self.create_publisher(Float32MultiArray, '/path_array', 10)
        
        self.qd = None  # goal (en coordenadas de mapa)
        self.q0 = None  # start (en coordenadas de mapa)
        self.robot_pose_world = None  # posición real del robot en el mundo
        
        self.map = None
        self.map_info = None
        self.last_map_hash = None
        self.current_start = None
        self.current_goal = None

    def position_callback(self, msg):
        x = msg.linear.x
        y = msg.linear.y

        self.robot_pose_world = (x, y)
        self.q0 = self.world_to_map(x, y)

        if self.q0 is None:
            self.get_logger().warn('⚠️ Current position is out of map bounds')
        else:
            self.get_logger().info(f'📍 Current position (start) set to: {self.q0}')
            if self.qd is not None:
                self.plan_path()

    def map_callback(self, msg):
        new_map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        new_hash = hashlib.md5(new_map_data).hexdigest()

        if new_hash == self.last_map_hash:
            return  # No cambios → no replanear

        self.map_info = msg.info
        self.map = new_map_data
        self.last_map_hash = new_hash

        self.get_logger().info('🗺️ Map updated. Replanning...')

        if self.qd is not None and self.q0 is not None:
            self.plan_path()

    def world_to_map(self, x, y):
        if self.map_info is None:
            self.get_logger().warn('⚠️ No map_info yet in world_to_map()')
            return None

        mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)

        if 0 <= mx < self.map_info.width and 0 <= my < self.map_info.height:
            return mx, my
        else:
            self.get_logger().warn('❌ World coordinates out of map bounds')
            return None

    def map_to_world(self, mx, my):
        x = (mx + 0.5) * self.map_info.resolution + self.map_info.origin.position.x
        y = (my + 0.5) * self.map_info.resolution + self.map_info.origin.position.y
        return x, y

    def desired_position_callback(self, msg):
        x_world = msg.x
        y_world = msg.y
        
        self.get_logger().info(f'Received desired goal: {x_world, y_world}')

        if self.map_info is None:
            self.get_logger().warn('⚠️ Map not received yet. Goal will be stored and used later.')
            self.qd = (x_world, y_world)
            return

        self.qd = self.world_to_map(x_world, y_world)

        if self.qd is not None:
            self.get_logger().info(f'🎯 Received desired goal in map coordinates: {self.qd}')
            if self.q0 is not None:
                self.plan_path()

    def is_occupied(self, x, y):
        return self.map[y, x] > 50

    def plan_path(self):
        if self.map_info is None or self.map is None:
            self.get_logger().warn('⚠️ Cannot plan path: map not ready.')
            return

        if self.qd is None or self.q0 is None:
            self.get_logger().warn('⚠️ Start or goal position not yet received.')
            return

        start = self.q0
        goal = self.qd

        if start is None or goal is None:
            self.get_logger().warn('🛑 Invalid start or goal.')
            return

        self.current_start = start
        self.current_goal = goal

        path = self.a_star_algorithm(start, goal)

        if path:
            self.publish_path(path)
        else:
            self.get_logger().info("❌ No path found")

    def a_star_algorithm(self, start, goal):
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_list:
            current = heapq.heappop(open_list)[1]
            if current == goal:
                return self.reconstruct_path(came_from, current)
            for neighbor in self.get_neighbors(current):
                if self.is_occupied(*neighbor):
                    continue
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))
        return None

    def get_neighbors(self, current):
        x, y = current
        neighbors = [(x + dx, y + dy) for dx in [-1, 0, 1] for dy in [-1, 0, 1]
                     if (dx != 0 or dy != 0)
                     and 0 <= x + dx < self.map.shape[1]
                     and 0 <= y + dy < self.map.shape[0]]
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
        path_msg.header.stamp = self.get_clock().now().to_msg()

        array_msg = Float32MultiArray()
        path_coords = []

        # Añadir posición real del robot si está disponible
        if self.robot_pose_world is not None:
            self.get_logger().info("Robot position received.")
            wx, wy = self.robot_pose_world
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            path_msg.poses.append(pose)
            path_coords.extend([wx, wy])
        else:
            self.get_logger().warn("⚠️ No real robot position available. Using map cell instead.")
            (wx, wy) = self.map_to_world(*path[0])
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            path_msg.poses.append(pose)
            path_coords.extend([wx, wy])

        # Resto del camino desde path[1:]
        for (x, y) in path[1:]:
            wx, wy = self.map_to_world(x, y)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            path_msg.poses.append(pose)
            path_coords.extend([wx, wy])

        self.path_pub.publish(path_msg)
        array_msg.data = path_coords
        self.pose_array_pub.publish(array_msg)

        self.get_logger().info(f'📤 Path array published with {len(path_coords)//2} points.')
        for i in range(0, len(path_coords), 2):
            self.get_logger().info(f'📍 Waypoint: ({path_coords[i]:.2f}, {path_coords[i+1]:.2f})')

def main():
    rclpy.init()
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
