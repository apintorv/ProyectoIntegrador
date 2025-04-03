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
        super().__init__('Control')

        # Initialize variables
        self.twist = Twist()
        self.bridge = CvBridge()
        self.kernel = np.ones((5, 5), np.uint8)
        self.vmax = 0.05  #0.05
        
        # PID controller parameters
        self.KP = 0.02  # Proportional gain 15
        # self.KI = 2  # Integral }
        self.KD = 0.001 # Derivative gain 5
        self.integral = 0.0
        self.previous_error = 0.0
        
        self.type = ''
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
        self.subscriber_flag_yolo = self.create_subscription(Bool, "/Detection_Flag", self.callback_flag_yolo,qos_profile)

        # Create publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.timer = self.create_timer(0.01, self.move)

    def callback_flag_yolo(self, msg):
        self.traffic_sign = msg.data

    def class_callback(self, msg):
        self.type = msg.data
        
    def move(self):
        
        if self.traffic_sign:
            self.twist.angular.z = self.z_yolo.data
            self.twist.linear.x = self.x_yolo.data
            self.get_logger().info(f'Pub {self.x_yolo}, {self.z_yolo}')
        else:
            self.twist.angular.z = self.z_line.data
            self.twist.linear.x = self.x_line.data
            self.get_logger().info(f'Pub {self.x_line}, {self.z_line}')

        self.publisher_.publish(self.twist)


    def image_callback(self, msg):  
        if self.type == 'Stop-pannels':
            self.twist.angular.z = 0.0
            self.twist.linear.x = 0.0  
            self.publisher_.publish(self.twist)      
        elif not self.traffic_sign:
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

            if contours:
                self.follow_line(contours, cx_a)

            time.sleep(0.05) 
            self.publisher_.publish(self.twist)
            self.get_logger().info(f'w: {self.twist.angular.z}, x: {self.twist.linear.x}')
        else:
            self.handle_traffic_sign()

    def handle_traffic_sign(self):
        if self.type == 'Stop-pannels':
            self.twist.angular.z = 0.0
            self.twist.linear.x = 0.0   
        elif self.type == 'Toutdroit-pannels':
            self.twist.angular.z = 0.0
            self.twist.linear.x = self.vmax
            time.sleep(0.8) 
        elif self.type == 'Derecha-Pannels':
            self.twist.angular.z = -0.02
            self.twist.linear.x = 0.0
            time.sleep(0.3) 
        # elif self.type == 'Roundpoint-pannels':
        #     self.twist.angular.z = 0.0
        #     self.twist.linear.x = 0.0
        elif self.type == 'Izquierda-pannels':
            # self.twist.angular.z = self.vmax
            # self.twist.linear.x = self.vmax
            pass
        elif self.type == 'Travaux-pannel':
            self.twist.linear.x /= 2
        elif self.type == 'Jaune-Sema':
            self.twist.angular.z /= 2
            self.twist.linear.x /= 2
        elif self.type == 'Vert-Sema':
            self.twist.linear.x = self.twist.linear.x
            
        self.get_logger().info(f'sign: {self.type}')
        self.type = ''         

    def follow_line(self, contours, cx_a):
        line_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(line_contour)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            self.error = cx - cx_a
            derivative = self.error - self.previous_error
            control = self.KP * self.error  + self.KD * derivative
            
            if abs(self.error) > 2:
                self.twist.angular.z = np.clip(-control, -0.05, 0.05)
            self.twist.linear.x = self.vmax
            
            self.previous_error = self.error

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()