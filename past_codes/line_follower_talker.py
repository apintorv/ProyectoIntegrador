import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
import cv2
from cv_bridge import CvBridge
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image,'/video_source/raw',self.image_callback,1)
        self.publisher_error = self.create_publisher(Float32, '/error', 10)
        # self.publisher_centro = self.create_publisher(Float32, '/cx_a', 10)
        self.publisher_contour_flag = self.create_publisher(Bool, '/flag', 10)
        # self.mask_image_pub = self.create_publisher(Image, '/Processed_Image', 10) #Descomentar para debug
        self.kernel = np.ones((5, 5), np.uint8)
        self.countour_detected = Bool()
        
    def image_callback(self, msg):        
        # Convertir el mensaje de imagen a un formato OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv_image = cv2.flip(cv_image, 0)
                
        height, width = cv_image.shape[:2]
        new_height = int(height * 0.9)
        new_width = int(width * 0.2)
        roi = cv_image[new_height:height, (width - new_width) // 2:width - (width - new_width) // 2]

        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        roi_blur = cv2.GaussianBlur(gray_roi, (15, 15), 0)
        eroded = cv2.erode(roi_blur, self.kernel, iterations=1)
        dilated = cv2.dilate(eroded, self.kernel, iterations=1)
        _, binary_roi = cv2.threshold(dilated, 100, 125, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(binary_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        height, width = roi.shape[:2]
        cx_a = width // 2
        # cy_a = height
        # cv2.circle(roi, (cx_a, cy_a), 5, (0, 255, 0), -1)
        # self.publisher_centro.publish(cx_a)

        # Si se encuentran contornos
        if contours:
            self.countour_detected = True
            contour_msg = Bool(self.countour_detected)
            self.publisher_contour_flag.publish(contour_msg)
            line_contour = max(contours, key=cv2.contourArea)

            # cv2.drawContours(roi, [line_contour], -1, (0, 0, 255), thickness=2)

            # Calcular el centro del contorno
            M = cv2.moments(line_contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                # cy = int(M['m01'] / M['m00'])
                # cv2.circle(roi, (cx, cy), 5, (0, 255, 0), -1)
                
                error = cx - cx_a
                # self.get_logger().info(f'Error: {error}')
                self.publisher_error.publish(error)

        else:
            self.countour_detected = False
            contour_msg = Bool(self.countour_detected)
            self.publisher_contour_flag.publish(contour_msg)

        # self.mask_image_pub.publish(self.bridge.cv2_to_imgmsg(roi, 'bgr8'))   #Descomentar para debug

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()