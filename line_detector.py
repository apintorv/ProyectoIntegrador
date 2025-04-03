import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import traceback  

class LineFollowerNode(Node):

    def __init__(self):
        super().__init__('line_detector')
        self.bridge = CvBridge()
        self.img_sub = self.create_subscription(Image, '/video_source/raw', self.img_callback, 1)
        self.box_origin_pub = self.create_publisher(Pose2D, '/box_center', 1)
        self.frame = self.create_publisher(Pose2D, '/frame_centes', 1)
        self.mask_image_pub = self.create_publisher(Image, '/Processed_Image', 10)
        self.timer = self.create_timer(1.0 / 60.0, self.process_img)
        self.frame = np.array([[]], dtype="uint8")
        self.cam_origin = Pose2D()
        self.box_origin = Pose2D()
        self.after_zero = []
        self.only_ones = []
                
        self.get_logger().info('Line Follower Node successfully initialized')

    def img_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except:
            self.get_logger().info('Failed to get an image')
            
    def process_img(self):
        try:
            if self.frame.size == 0:
                self.get_logger().info('Empty image array')
                return

            dst = self.frame.astype(np.uint8)
            dst = cv2.flip(dst,0)
            
            height = dst.shape[0]
            width = dst.shape[1]
            
            # Define the region of interest in the image
            idh = int(height / 4) * 3
            idw = int(width / 5)
            fdh = height
            fdw = int(idw * 4)
            dst = dst[idh:fdw, idw:fdw]  # Crop the image to the region of interest

            new_height, new_width = dst.shape[:2]  # Get the dimensions of the cropped image
            
            # new_height = int(height * 0.85)
            # new_width = int(width * 0.65)
            #dst = dst[new_height:height, 0+(width-new_width)//2:width-(width-new_width)//2]

            gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (11, 11), 0)
            val, thresh = cv2.threshold(blur, 110, 255, cv2.THRESH_BINARY)

            origin_x = int(new_width / 2)
            origin_y = int((height - new_height) / 2)
            cv2.circle(dst, (origin_x, origin_y), 2, (0, 255, 0), 1)
            self.cam_origin.x = float(origin_x)  # Convertir a flotante
            self.cam_origin.y = float(origin_y)  # Convertir a flotante

            threshold = 150
            nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(thresh, connectivity=8)
            sizes = stats[1:, -1]
            nb_components = nb_components - 1
            img = np.zeros((output.shape), dtype=np.uint8)
            for i in range(0, nb_components):
                if sizes[i] >= threshold:
                    img[output == i + 1] = 255

            contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
            cv2.drawContours(dst, contours, -1, (255, 0, 0), 2)
            count = len(contours)

            if count >= 1:
                if count == 1:
                    self.only_ones.append(1)
                else:
                    self.only_ones = []

                if len(self.after_zero) >= 1 and self.after_zero[0] == 0:
                    self.after_zero.append(count)

                if 3 in self.after_zero or 4 in self.after_zero:
                    self.after_zero = []
                else:
                    pass

                if len(self.after_zero) >= 65:
                    self.after_zero = []

                if len(self.only_ones) >= 85:
                    self.only_ones = []
                else:
                    pass

                c = max(contours, key=cv2.contourArea)
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                cv2.drawContours(dst, [box], 0, (0, 0, 255), 2)

                box_x = int(rect[0][0])
                box_y = int(rect[0][1])
                box_theta = rect[2]

                cv2.circle(dst, (box_x, box_y), 2, (0, 0, 255), 2)

                self.box_origin.x = float(box_x)  # Convertir a flotante
                self.box_origin.y = float(box_y)  # Convertir a flotante
                self.box_origin.theta = box_theta
                self.box_origin_pub.publish(self.box_origin)
                self.get_logger().info(f'Pub {self.box_origin.x}, {self.box_origin.y}, , {self.box_origin.theta}')
                
                self.mask_image_pub.publish(self.bridge.cv2_to_imgmsg(dst, 'bgr8'))


            else:
                self.after_zero = []
                self.after_zero.append(0)
                self.only_ones = []

        except Exception as e:
            self.get_logger().info('Error al procesar la imagen:')
            self.get_logger().info(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()