import rclpy
from rclpy.node import Node
import math
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image

class colors(Node):
    def __init__(self):
        super().__init__('Color_Detector')
        self.publisher = self.create_publisher(Image, 'color', 10)
        self.subscriber_wl = self.create_subscription(Image, '/video_source/raw', self.callback_image, 10)
        self.bridge = CvBridge()
        self.valid_img = False
        self.actual = ""
        self.previo = ""
        self.frame = np.ndarray((720, 1280, 3))
        
        self.redBajo1 = np.array([0, 100, 20], np.uint8)
        self.redAlto1 = np.array([8, 255, 255], np.uint8)
        self.redBajo2 = np.array([175, 100, 20], np.uint8)
        self.redAlto2 = np.array([179, 255, 255], np.uint8)

        self.verdeBajo = np.array([40, 100, 20], np.uint8)
        self.verdeAlto = np.array([70, 255, 255], np.uint8)

        self.amarilloBajo = np.array([15, 100, 20], np.uint8)
        self.amarilloAlto = np.array([35, 255, 255], np.uint8)
        
        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)
        self.get_logger().info('Talker node successfully initialized')

    def callback_image(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg,'bgr8')
            self.valid_img = True
        except:
            self.get_logger().info('Failed to get an image')

    def timer_callback(self):
        try:
            if self.valid_img:
                self.get_logger().info('Processing image to detect color')
                
                frameHSV = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

                self.mask_red = cv2.add(cv2.inRange(frameHSV, self.redBajo1, self.redAlto1), cv2.inRange(frameHSV, self.redBajo2, self.redAlto2))
                self.maskGreen = cv2.inRange(frameHSV, self.verdeBajo, self.verdeAlto)
                self.maskYellow = cv2.inRange(frameHSV, self.amarilloBajo, self.amarilloAlto)
                
                self.get_logger().info('Mask created')

                self.detect_callback(self.mask_red, self.maskGreen, self.maskYellow)
                
                self.get_logger().info(self.actual)
                
                # Máscaras visuales
                maskCombined = cv2.add(self.mask_red, cv2.add(self.maskGreen, self.maskYellow))
                maskCombinedVis = cv2.bitwise_and(self.frame, self.frame, mask=maskCombined)
                
                self.publisher.publish(self.bridge.cv2_to_imgmsg(maskCombinedVis, 'bgr8'))
                
                if self.actual:
                    #self.publisher.publish(self.actual)
                    self.get_logger().info(f'Detected color: {self.actual}')
                    self.previo = self.actual
                    
                self.valid_img = False

        except:
            self.get_logger().info('Failed')

    def detect_callback(self, mask_red, maskGreen, maskYellow):
        # Contar píxeles en cada máscara
        pixelsRed = cv2.countNonZero(mask_red)
        pixelsVerde = cv2.countNonZero(maskGreen)
        pixelsAmarillo = cv2.countNonZero(maskYellow)

        # Determinar el color predominante
        if pixelsRed > pixelsVerde and pixelsRed > pixelsAmarillo:
            self.actual =  "Red"
        elif pixelsVerde > pixelsRed and pixelsVerde > pixelsAmarillo:
            self.actual =  "Green"
        elif pixelsAmarillo > pixelsRed and pixelsAmarillo > pixelsVerde:
            self.actual =  "Yellow"
        else:
            self.actual = "No dominant color"
        

def main(args=None):
    rclpy.init(args=args)
    m_p = colors()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

