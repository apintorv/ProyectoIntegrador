from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge

bridge = CvBridge()

class TrafficSignsRecognition(Node):

    def __init__(self):
        super().__init__('traffic_signs_recognition')
        
        # Cargar modelo YOLO
        self.model = YOLO('/home/puzzlebot/ros2_ws/src/open_cv_example/open_cv_example/best.pt')

        # Crear suscripción y publicador de ROS2
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.image_callback, 1)
        self.yolov8_pub = self.create_publisher(String, "/Yolov8_Inference", 1)
        
    def image_callback(self, msg):
        # Convertir mensaje de imagen de ROS a imagen de OpenCV
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        down = cv2.pyrDown(img)
        # Voltear la imagen si es necesario (aquí se voltea verticalmente)
        flipped_img = cv2.flip(down, 0)
        # Realizar inferencia con el modelo YOLO
        results = self.model(flipped_img)

        for r in results:
            boxes = r.boxes
            for box in boxes:
                c = int(box.cls)
                class_name = self.model.names[c]
                
                # Loggear la clase detectada
                self.get_logger().info(f'Detected class: {class_name}')
                # Publicar cada clase detectada
                type_msg = String()
                type_msg.data = class_name
                self.yolov8_pub.publish(type_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignsRecognition()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
