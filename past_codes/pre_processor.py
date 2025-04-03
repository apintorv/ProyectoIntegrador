import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class Preprocessor(Node):
    def __init__(self):
        super().__init__('line_follower')
 
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.image_callback, 1)
        self.mask_image_pub = self.create_publisher(Image, '/Preprocessed_Image', 10)
        self.kernel = np.ones((5, 5), np.uint8)

        # Create a timer to publish at 1 Hz
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.latest_image = None

    def image_callback(self, msg):
        # Store the latest image message
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def timer_callback(self):
        if self.latest_image is None:
            return

        # Convert the message to an OpenCV image
        cv_image = self.latest_image
        cv_image = cv2.flip(cv_image, 0)
        down = cv2.pyrDown(cv_image)

        height, width = down.shape[:2]

        # Definir el número de píxeles a recortar desde la parte superior
        num_pixels_to_cut = 70  # Ajusta este valor según tus necesidades

        # Extraer la ROI excluyendo los primeros píxeles desde la parte superior
        roi = down[num_pixels_to_cut:height, 0:width]

        # Publish the processed image
        self.mask_image_pub.publish(self.bridge.cv2_to_imgmsg(roi, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = Preprocessor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
