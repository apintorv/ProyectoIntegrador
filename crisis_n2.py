import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class TrafficHandler(Node):
    def __init__(self):
        super().__init__('Traffic_Handler')
        
        # Define QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5
        )
        
        
        self.sign_type = ''
        
        self.subscriber_class = self.create_subscription(String, '/Yolov8_Inference', self.class_callback, qos_profile)
        
        self.publisher_x_yolo = self.create_publisher(Float32, 'linea_yolo', 10)
        self.publisher_z_yolo = self.create_publisher(Float32, 'angular_yolo', 10)


    def class_callback(self, msg): 
        self.sign_type = msg.data
        
    def handle_traffic_sign(self):
        if self.sign_type == 'Stop-pannels':
            z = 0.0
            x = 0.0
            self.publisher_z.publish(z)  
            self.publisher_x.publish(x)
        elif self.sign_type == 'Rouge-Sema':
            z = 0.0
            x = 0.0
            self.publisher_z.publish(z)  
            self.publisher_x.publish(x)
        elif self.sign_type == 'Toutdroit-pannels':
            z = 0.0
            x = 0.02
            self.publisher_z.publish(z)  
            self.publisher_x.publish(x)
        elif self.sign_type == 'Derecha-Pannels':
            z = -0.02
            x = 0.0
            self.publisher_z.publish(z)  
            self.publisher_x.publish(x)
        elif self.sign_type == 'Vert-Sema':
            z = 0.0
            x = 0.02
            self.publisher_z.publish(z)  
            self.publisher_x.publish(x)
        elif self.sign_type == 'Travaux-pannel': 
            z = 0.02
            x = 0.02
            self.publisher_z.publish(z)  
            self.publisher_x.publish(x)
        elif self.sign_type == 'Jaune-Sema': 
            z = 0.02
            x = 0.02
            self.publisher_z.publish(z)  
            self.publisher_x.publish(x)

        self.get_logger().info(f'Pub {x}, {z}')
        self.get_logger().info(f'sign: {self.sign_type}')
        self.sign_type = ''
        
def main(args=None):
    rclpy.init(args=args)
    node = TrafficHandler()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
