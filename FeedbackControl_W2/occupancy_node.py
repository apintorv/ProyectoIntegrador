import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan

class OccupancyMapNode(Node):
    def __init__(self):
        super().__init__('occupancy_map_node')
        
        # Parameters
        self.declare_parameter('map_size', 5)
        self.declare_parameter('max_range', 1.0)
        self.declare_parameter('decay_rate', 0.5)
        self.declare_parameter('update_interval', 1.0)  # seconds between updates
        
        # Variables
        self.lidar_coords = []
        self.map_size = self.get_parameter('map_size').value
        self.max_range = self.get_parameter('max_range').value
        self.decay_rate = self.get_parameter('decay_rate').value
        
        # Subscriber
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Default LIDAR topic
            self.lidar_callback,
            10
        )
        
        # Timer for periodic updates
        self.create_timer(
            self.get_parameter('update_interval').value,
            self.log_occupancy_map
        )
    
    def lidar_callback(self, msg):
        """Convert LaserScan data to Cartesian coordinates"""
        self.lidar_coords = []
        
        for i, distance in enumerate(msg.ranges):
            # Skip invalid measurements
            if distance < msg.range_min or distance > msg.range_max:
                continue
                
            # Convert polar to Cartesian coordinates
            angle = msg.angle_min + i * msg.angle_increment
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)
            
            # Scale and discretize coordinates to fit our map
            scaled_x = int((x + self.map_size/2) % self.map_size)
            scaled_y = int((y + self.map_size/2) % self.map_size)
            
            self.lidar_coords.append((scaled_x, scaled_y))
    
    def create_occupancy_map(self):
        """Create occupancy probability map from LIDAR coordinates"""
        occupancy_map = np.zeros((self.map_size, self.map_size))
        
        for (x, y) in self.lidar_coords:
            if 0 <= x < self.map_size and 0 <= y < self.map_size:
                occupancy_map[y, x] = max(occupancy_map[y, x], self.max_range)
                
                # Apply decay to neighboring cells
                for di in [-1, 0, 1]:
                    for dj in [-1, 0, 1]:
                        if di == 0 and dj == 0:
                            continue
                        ni, nj = y + di, x + dj
                        if 0 <= ni < self.map_size and 0 <= nj < self.map_size:
                            new_val = self.max_range * (1.0 - self.decay_rate)
                            occupancy_map[ni, nj] = max(occupancy_map[ni, nj], new_val)
        
        return occupancy_map
    
    def log_occupancy_map(self):
        """Log the occupancy matrix to console"""
        if not self.lidar_coords:
            self.get_logger().info("Waiting for LIDAR data...")
            return
            
        occupancy_map = self.create_occupancy_map()
        
        # Format the matrix for logging
        matrix_str = "\nOccupancy Matrix:\n"
        for row in occupancy_map:
            matrix_str += " ".join(f"{val:.2f}" for val in row) + "\n"
        
        self.get_logger().info(matrix_str)

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()