import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy

from cv_bridge import CvBridge
import cv2
import imutils

class ColorDetector(Node):

    def __init__(self):
        super().__init__('detector')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.br = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos)

    def image_callback(self, msg):
        self.get_logger().info('Image Frame Id: "%s"' % msg.header.frame_id)
        
        current_frame = self.br.imgmsg_to_cv2(msg, "bgr8")

        self.detector(current_frame)

    def detector(self, current_frame):

        blackLower = (0, 0, 0) 
        blackUpper = (5,50,50)
        redLower = (0, 50, 50)
        redUpper = (5, 255, 255)
        yellowLower = (25, 50, 50) 
        yellowUpper = (35, 255, 255)
        greenLower = (50, 50, 50) 
        greenUpper = (70, 255, 255)
        blueLower = (100, 50, 50) 
        blueUpper = (130, 255, 255)
        magentaLower = (125, 50, 50) 
        magentaUpper = (150, 255, 255)

        blurred = cv2.GaussianBlur(current_frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask_blk = cv2.inRange(hsv, blackLower, blackUpper)
        mask_blk = cv2.erode(mask_blk, None, iterations=2)
        mask_blk = cv2.dilate(mask_blk, None, iterations=2)

        mask_r = cv2.inRange(hsv, redLower, redUpper)
        mask_r = cv2.erode(mask_r, None, iterations=2)
        mask_r = cv2.dilate(mask_r, None, iterations=2)

        mask_y = cv2.inRange(hsv, yellowLower, yellowUpper)
        mask_y = cv2.erode(mask_y, None, iterations=2)
        mask_y = cv2.dilate(mask_y, None, iterations=2)

        mask_g = cv2.inRange(hsv, greenLower, greenUpper)
        mask_g = cv2.erode(mask_g, None, iterations=2)
        mask_g = cv2.dilate(mask_g, None, iterations=2)

        mask_blu = cv2.inRange(hsv, blueLower, blueUpper)
        mask_blu = cv2.erode(mask_blu, None, iterations=2)
        mask_blu = cv2.dilate(mask_blu, None, iterations=2)

        mask_m = cv2.inRange(hsv, magentaLower, magentaUpper)
        mask_m = cv2.erode(mask_m, None, iterations=2)
        mask_m = cv2.dilate(mask_m, None, iterations=2)

        cnts_blk = cv2.findContours(mask_blk.copy(), cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE)
        cnts_r = cv2.findContours(mask_r.copy(), cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE)
        cnts_y = cv2.findContours(mask_y.copy(), cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE)
        cnts_g = cv2.findContours(mask_g.copy(), cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE)
        cnts_blu = cv2.findContours(mask_blu.copy(), cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE)
        cnts_m = cv2.findContours(mask_m.copy(), cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE)
        
        cnts_blk = imutils.grab_contours(cnts_blk)
        cnts_r = imutils.grab_contours(cnts_r)
        cnts_y = imutils.grab_contours(cnts_y)
        cnts_g = imutils.grab_contours(cnts_g)
        cnts_blu = imutils.grab_contours(cnts_blu)
        cnts_m = imutils.grab_contours(cnts_m)

        center = None
        c = 0
        radius = 0

        global black_ball, red_ball, yellow_ball, green_ball, blue_ball, magenta_ball

        if len(cnts_blk) > 0:
            c = max(cnts_blk, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            cv2.circle(current_frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(current_frame, center, 5, (0, 0, 255), -1)
            print ("Black ball detected.\n")

        if len(cnts_r) > 0:
            c = max(cnts_r, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            cv2.circle(current_frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(current_frame, center, 5, (0, 0, 255), -1)
            print ("Red ball detected.\n")

        if len(cnts_y) > 0:
            c = max(cnts_y, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            cv2.circle(current_frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(current_frame, center, 5, (0, 0, 255), -1)
            print ("Yellow ball detected.\n")

        if len(cnts_g) > 0:
            c = max(cnts_g, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            cv2.circle(current_frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(current_frame, center, 5, (0, 0, 255), -1)
            print ("Green ball detected.\n")

        if len(cnts_blu) > 0:
            c = max(cnts_blu, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            cv2.circle(current_frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(current_frame, center, 5, (0, 0, 255), -1)
            print ("Blue ball detected.\n")

        if len(cnts_m) > 0:
            c = max(cnts_m, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            cv2.circle(current_frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(current_frame, center, 5, (0, 0, 255), -1)
            print ("Magenta ball detected.\n")

        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)
        c = 0







def main(args=None):
    rclpy.init(args=args)
    color_detector = ColorDetector()
    rclpy.spin(color_detector)
    color_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()