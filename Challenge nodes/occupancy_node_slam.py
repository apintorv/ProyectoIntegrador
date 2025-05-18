import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer
import numpy as np
import math

MAP_SIZE = 28  # 28x28 cells
RESOLUTION = 0.15  # meters per cell (15cm)
MAX_PROB = 1.0
MIN_PROB = 0.0

class MixedMapNode(Node):
    def __init__(self):
        super().__init__('mixed_map_node')

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Publisher
        self.mixed_map_pub = self.create_publisher(OccupancyGrid, '/mixed_map', 10)

        # TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Maps
        self.local_map = np.full((MAP_SIZE, MAP_SIZE), 0.5)  # Local occupancy map
        self.static_map = np.full((MAP_SIZE, MAP_SIZE), 0.5)  # Latest SLAM map

        # Map origin (centered at 0,0)
        self.origin_x = -MAP_SIZE // 2
        self.origin_y = -MAP_SIZE // 2

        # Timer to publish the mixed map
        self.timer = self.create_timer(0.5, self.publish_mixed_map)

    def map_callback(self, msg: OccupancyGrid):
        # Copy relevant part of global map into our fixed-size local grid
        map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        resolution_ratio = int(msg.info.resolution / RESOLUTION)

        start_x = msg.info.width // 2 - MAP_SIZE // 2
        start_y = msg.info.height // 2 - MAP_SIZE // 2

        for i in range(MAP_SIZE):
            for j in range(MAP_SIZE):
                mx = start_y + j
                my = start_x + i
                if 0 <= mx < msg.info.height and 0 <= my < msg.info.width:
                    val = map_data[mx, my]
                    self.static_map[j, i] = val / 100.0 if val >= 0 else 0.5
                else:
                    self.static_map[j, i] = 0.5

    def scan_callback(self, msg: LaserScan):
        try:
            tf = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"Transform not available: {e}")
            return

        dx = tf.transform.translation.x
        dy = tf.transform.translation.y
        yaw = self.get_yaw_from_quaternion(tf.transform.rotation)

        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x_lidar = r * math.cos(angle)
                y_lidar = r * math.sin(angle)

                # Transform to map frame
                x = math.cos(yaw) * x_lidar - math.sin(yaw) * y_lidar + dx
                y = math.sin(yaw) * x_lidar + math.cos(yaw) * y_lidar + dy

                self.update_local_map(dx, dy, x, y)
            angle += msg.angle_increment

    def update_local_map(self, x0, y0, x1, y1):
        def to_cell(x, y):
            i = int(x / RESOLUTION) + MAP_SIZE // 2
            j = MAP_SIZE // 2 - int(y / RESOLUTION)
            return i, j

        x0_i, y0_i = to_cell(x0, y0)
        x1_i, y1_i = to_cell(x1, y1)

        points = self.bresenham(x0_i, y0_i, x1_i, y1_i)
        for i, j in points[:-1]:
            if 0 <= i < MAP_SIZE and 0 <= j < MAP_SIZE:
                self.local_map[j, i] = max(self.local_map[j, i] - 0.1, MIN_PROB)
        i, j = points[-1]
        if 0 <= i < MAP_SIZE and 0 <= j < MAP_SIZE:
            self.local_map[j, i] = min(self.local_map[j, i] + 0.1, MAX_PROB)

    def bresenham(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points.append((x1, y1))
        return points

    def publish_mixed_map(self):
        # Blend local and static map with weighted average (50/50)
        mixed_map = 0.5 * self.local_map + 0.5 * self.static_map

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = RESOLUTION
        msg.info.width = MAP_SIZE
        msg.info.height = MAP_SIZE
        msg.info.origin.position.x = self.origin_x * RESOLUTION
        msg.info.origin.position.y = self.origin_y * RESOLUTION
        msg.data = [int(p * 100) if 0.0 <= p <= 1.0 else -1 for p in mixed_map.flatten()]
        self.mixed_map_pub.publish(msg)
        
        
        output = ''
        for row in mixed_map:
            line = ' '.join([f"{p:.2f}" for p in row])  # Print the raw probabilities
            output += line + '\n'
        self.get_logger().info('\n' + output)

        # Also print the raw probability values in a clearer format
        prob_output = '\n'.join([', '.join([f"{p:.2f}" for p in row]) for row in mixed_map])
        self.get_logger().info(f"\nProbabilities:\n{prob_output}")

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = MixedMapNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
