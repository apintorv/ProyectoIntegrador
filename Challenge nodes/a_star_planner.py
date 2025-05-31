import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Vector3, Pose2D, Twist
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import numpy as np
import heapq
import hashlib
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from tf_transformations import euler_from_quaternion  # If you use tf_transformations


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('a_star_planner')
        

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(OccupancyGrid, '/map_updated', self.map_callback, qos)
        self.create_subscription(Vector3, '/qd', self.desired_position_callback, 10)
        # self.create_subscription(Twist, '/pose', self.position_callback, 10)
        self.create_subscription(PoseStamped, '/pose_stamped', self.position_callback, 10)
        #self.create_subscription(Pose2D, '/pose_kalman', self.position_callback, 10)

        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.pose_array_pub = self.create_publisher(Float32MultiArray, '/path_array', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_marker', 10)


        self.qd = None
        self.q0 = None
        self.qd_world = None
        self.q0_world = None
        self.robot_pose_world = None

        self.map = None
        self.map_info = None
        self.last_map_hash = None
        self.current_start = None
        self.current_goal = None

        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_map_yaw(self):
        q = self.map_info.origin.orientation
        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        return yaw

    def position_callback(self, msg):
        # x = msg.linear.x
        # y = msg.linear.y

        x = msg.pose.position.x
        y = msg.pose.position.y

        #self.get_logger().info(f'x: {x}  y: {y}')
        self.robot_pose_world = (x, y)
        self.q0_world = (x, y)

        if self.map_info is not None:
            self.q0 = self.world_to_map(x, y)
            if self.q0 is None:
                self.get_logger().warn('⚠️ Current position is out of map bounds')
            else:
                self.get_logger().info(f'📍 Current position (start) set to: {self.q0}')
                if self.qd is not None:
                    self.plan_path()


    def desired_position_callback(self, msg):
        x_world, y_world = msg.x, msg.y
        self.qd_world = (x_world, y_world)
        self.get_logger().info(f'🎯 Received desired goal in world: {x_world}, {y_world}')

        if self.map_info is not None:
            self.qd = self.world_to_map(x_world, y_world)
            if self.qd is not None:
                self.get_logger().info(f'🎯 Converted desired goal to map coords: {self.qd}')
                if self.q0 is not None:
                    self.plan_path()
                    
        if self.qd is not None:
            # self.get_logger().info(f'🗺️ Goal in map coordinates (qd): {self.qd}')
            
            # Convert map coordinates back to world for visualization
            wx, wy = self.map_to_world(*self.qd)

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = wx
            goal_pose.pose.position.y = wy
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.w = 1.0  # No rotation

            self.goal_pose_pub.publish(goal_pose)
            self.get_logger().info("📍 Published goal marker for RViz.")


    def map_callback(self, msg):
        new_map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        new_hash = hashlib.md5(new_map_data).hexdigest()

        if new_hash == self.last_map_hash:
            return

        self.map_info = msg.info
        self.map = new_map_data
        self.last_map_hash = new_hash

        if self.q0 is None and self.q0_world is not None:
            self.q0 = self.world_to_map(*self.q0_world)
        if self.qd is None and self.qd_world is not None:
            self.qd = self.world_to_map(*self.qd_world)

        self.get_logger().info('🗺️ Map updated. Replanning...')

        if self.qd is not None and self.q0 is not None:
            self.plan_path()

    def world_to_map(self, x, y):
        if self.map_info is None:
            return None

        mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)

        if 0 <= mx < self.map_info.width and 0 <= my < self.map_info.height:
            return mx, my
        else:
            self.get_logger().warn('❌ World coordinates out of map bounds')
            return None
        # origin = self.map_info.origin.position
        # resolution = self.map_info.resolution
        # q = self.map_info.origin.orientation
        # quat = [q.x, q.y, q.z, q.w]

        # roll, pitch, yaw = euler_from_quaternion(quat)

        # # Translate
        # dx = x - origin.x
        # dy = y - origin.y

        # # Rotate by -yaw to align
        # cos_yaw = np.cos(-yaw)
        # sin_yaw = np.sin(-yaw)
        # rx = dx * cos_yaw - dy * sin_yaw
        # ry = dx * sin_yaw + dy * cos_yaw

        # mx = int(rx / resolution)
        # my = int(ry / resolution)

        # if 0 <= mx < self.map_info.width and 0 <= my < self.map_info.height:
        #     return mx, my
        # else:
        #     return None


    def map_to_world(self, mx, my):
        x = (mx + 0.5) * self.map_info.resolution + self.map_info.origin.position.x
        y = (my + 0.5) * self.map_info.resolution + self.map_info.origin.position.y
        return x, y
        # origin = self.map_info.origin.position
        # resolution = self.map_info.resolution
        # q = self.map_info.origin.orientation
        # quat = [q.x, q.y, q.z, q.w]

        # roll, pitch, yaw = euler_from_quaternion(quat)

        # # Calculate world coordinates
        # rx = mx * resolution
        # ry = my * resolution

        # # Rotate by yaw
        # cos_yaw = np.cos(yaw)
        # sin_yaw = np.sin(yaw)
        # dx = rx * cos_yaw - ry * sin_yaw
        # dy = rx * sin_yaw + ry * cos_yaw

        # x = origin.x + dx
        # y = origin.y + dy
        # return x, y



    def is_occupied(self, x, y):
        return self.map[y, x] > 50

    def is_too_close_to_obstacle(self, cell):
        x, y = cell
        buffer_radius = 10
        for dx in range(-buffer_radius, buffer_radius + 1):
            for dy in range(-buffer_radius, buffer_radius + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.map.shape[1] and 0 <= ny < self.map.shape[0]:
                    if self.map[ny, nx] > 90:
                        return True
        return False

    def plan_path(self):
        if self.map_info is None or self.map is None:
            self.get_logger().warn('⚠️ Cannot plan path: map not ready.')
            return
        if self.qd is None or self.q0 is None:
            self.get_logger().warn('⚠️ Start or goal position not yet received.')
            return

        self.get_logger().info(f'🧭 Planning path from {self.q0} to {self.qd}')
        start = self.q0
        goal = self.qd
        self.current_start = start
        self.current_goal = goal

        path = self.a_star_algorithm(start, goal)
        if path:
            self.publish_path(path)
        else:
            self.get_logger().info("❌ No path found")

    def a_star_algorithm(self, start, goal, custom_map=None):
        grid = custom_map if custom_map is not None else self.map
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
                if grid[neighbor[1], neighbor[0]] > 50:
                    continue
                if self.is_too_close_to_obstacle(neighbor):
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

        if self.robot_pose_world is not None:
            self.get_logger().info("✅ Robot world position used.")
            wx, wy = self.robot_pose_world
        else:
            wx, wy = self.map_to_world(*path[0])
            self.get_logger().warn("⚠️ No real robot position available. Using map cell.")

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wx
        pose.pose.position.y = wy
        path_msg.poses.append(pose)
        path_coords.extend([wx, wy])

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

        self.get_logger().info(f'📤 Published path with {len(path_coords) // 2} waypoints.')

    def is_path_obstructed(self, new_map):
        if self.current_start is None or self.current_goal is None:
            return True
        path = self.a_star_algorithm(self.current_start, self.current_goal, custom_map=new_map)
        if path is None:
            return True
        for (x, y) in path:
            if self.is_too_close_to_obstacle((x, y)):
                self.get_logger().info(f"⚠️ Path is too close to obstacle at ({x}, {y})")
                return True
        return False

    def timer_callback(self):
        #self.get_logger().info("⏱️ Timer callback triggered.")

        if self.q0:
            self.get_logger().info(f"Current start: {self.q0}")
        if self.qd:
            self.get_logger().info(f"Current goal: {self.qd}")

        if self.map is not None and self.current_start and self.current_goal:
            if self.is_path_obstructed(self.map):
                self.get_logger().warn("🚧 Path obstructed — triggering replanning.")
                self.plan_path()


def main():
    rclpy.init()
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
