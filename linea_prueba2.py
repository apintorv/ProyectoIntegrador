import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
import cv2
import time
from cv_bridge import CvBridge
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')

        # Initialize variables
        self.twist = Twist()
        self.bridge = CvBridge()
        self.kernel = np.ones((5, 5), np.uint8)
        self.vmax = 0.05
        self.KP = 0.5
        self.type = ''
        self.color = ''
        self.traffic_sign = False

        # Define QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10
        )

        # Create subscriptions
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.image_callback, 1)
        self.subscriber_class = self.create_subscription(String, '/Yolov8_Inference', self.class_callback, qos_profile)
        self.subscriber_color = self.create_subscription(String, 'color', self.callback_color, qos_profile)
        self.subscriber_flag = self.create_subscription(Bool, 'flag', self.callback_flag, qos_profile)

        # Create publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def callback_flag(self, msg):
        self.traffic_sign = msg.data

    def class_callback(self, msg):
        self.type = msg.data

    def callback_color(self, msg):
        self.color = msg.data

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv_image = cv2.flip(cv_image, 0)
        cv_image = cv2.flip(cv_image, 1)

        # Define region of interest (ROI)
        height, width = cv_image.shape[:2]
        roi_height = int(height * 0.9)
        roi_width = int(width * 0.2)
        roi = cv_image[roi_height:height, (width - roi_width) // 2:width - (width - roi_width) // 2]

        # Process ROI to detect line
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        roi_blur = cv2.GaussianBlur(gray_roi, (15, 15), 0)
        eroded = cv2.erode(roi_blur, self.kernel, iterations=1)
        dilated = cv2.dilate(eroded, self.kernel, iterations=1)
        _, binary_roi = cv2.threshold(dilated, 100, 125, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(binary_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Center of the ROI
        height, width = roi.shape[:2]
        cx_a = width // 2

        # Traffic sign handling
        if self.type == 'Stop-pannels' or self.color == 'Red':
            self.twist.angular.z = 0.0
            self.twist.linear.x = 0.0
        else:
            if self.traffic_sign or self.color:
                self.handle_traffic_sign()
                self.traffic_sign = False
            elif contours:
                self.follow_line(contours, cx_a)
            else:
                self.no_line_detected()

        time.sleep(0.4)
        
        self.publisher_.publish(self.twist)
        self.get_logger().info(f'w: {self.twist.angular.z}, x: {self.twist.linear.x}')
        self.get_logger().info(f'sign: {self.type}, color: {self.color}')

    def handle_traffic_sign(self):
        if self.type == 'Toutdroit-pannels':
            # self.get_logger().info('Toutdroit-pannels')
            self.twist.angular.z = 0.0
            self.twist.linear.x = self.vmax
        elif self.type == 'Derecha-Pannels':
            # self.get_logger().info('Derecha-Pannels')
            # self.twist.angular.z = -self.vmax
            # self.twist.linear.x = self.vmax
            pass
        elif self.type == 'Izquierda-pannels':
            # self.get_logger().info('Izquierda-pannels')
            # self.twist.angular.z = self.vmax
            # self.twist.linear.x = self.vmax
            pass
        elif self.type == 'Travaux-pannel':
            # self.get_logger().info('Travaux-pannel')
            self.twist.angular.z /= 2
            self.twist.linear.x /= 2
        elif self.color == 'Yellow':
                # self.get_logger().info('Yellow')
                self.twist.angular.z /= 2
        elif self.color == 'Green':
            self.twist.angular.z = 0.0
        
        self.color = ''
        self.type = ''

    def follow_line(self, contours, cx_a):
        line_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(line_contour)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            error = cx - cx_a
            if abs(error) > 3:
                self.twist.angular.z = np.clip(-self.KP * error, -self.vmax, self.vmax)
            
            self.twist.linear.x = self.vmax
        else:
            self.twist.linear.x = self.vmax

    def no_line_detected(self):
        self.get_logger().info('No contours/signal detected')
        self.twist.angular.z = self.vmax
        self.twist.linear.x = self.vmax

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
