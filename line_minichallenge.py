import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import cv2
from cv_bridge import CvBridge
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image,'/video_source/raw',self.image_callback,1)
        
        self.KP = 0.5
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.mask_image_pub = self.create_publisher(Image, '/Processed_Image', 10)
        self.kernel = np.ones((5, 5), np.uint8)
        
    def image_callback(self, msg):
        # Convertir el mensaje de imagen a un formato OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv_image = cv2.flip(cv_image, 0)
        # cv_image = cv2.flip(cv_image, 1)
        
        twist = Twist()
        
        height, width = cv_image.shape[:2]
        new_height = int(height * 0.9)
        new_width = int(width * 0.2)
        roi = cv_image[new_height:height, (width - new_width) // 2:width - (width - new_width) // 2]

        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        #Use adaptive threshold to handle varying lighting conditions
        # adaptive_thresh = cv2.adaptiveThreshold(gray_roi, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
        # # Improve line detection using Canny edge detector
        # edges = cv2.Canny(adaptive_thresh, 50, 150)
        # # Dilate the edges to make the line more prominent
        # dilated_edges = cv2.dilate(edges, self.kernel, iterations=1)
        # contours, _ = cv2.findContours(dilated_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        #DESCOMENTAR PARA IR A ORIGINAL
        roi_blur = cv2.GaussianBlur(gray_roi, (15, 15), 0)
        eroded = cv2.erode(roi_blur, self.kernel, iterations=1)
        dilated = cv2.dilate(eroded, self.kernel, iterations=1)
        _, binary_roi = cv2.threshold(dilated, 100, 125, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(binary_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        height, width = roi.shape[:2]
        cx_a = width // 2
        cy_a = height
        cv2.circle(roi, (cx_a, cy_a), 5, (0, 255, 0), -1)

        # Si se encuentran contornos
        if contours:
            line_contour = max(contours, key=cv2.contourArea)

            cv2.drawContours(roi, [line_contour], -1, (0, 0, 255), thickness=2)

            # Calcular el centro del contorno
            M = cv2.moments(line_contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(roi, (cx, cy), 5, (0, 255, 0), -1)
                
                error = cx - cx_a
                self.get_logger().info(f'Error: {error}')

                if error > 3 and error > -3:
                    w = - self.KP * error 
                
                    twist.angular.z = np.clip(w, -0.1, 0.1)
                    
                    self.get_logger().info(f'w: {twist.angular.z}')
                    
                twist.linear.x = 0.1  # Velocidad constante hacia adelante
                self.publisher_.publish(twist)
        else:
            twist.angular.z = 0.08
            twist.linear.x = 0.05  # Velocidad constante hacia adelante
        
        self.mask_image_pub.publish(self.bridge.cv2_to_imgmsg(roi, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()