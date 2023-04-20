import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from rclpy.qos import QoSProfile, ReliabilityPolicy

class ColorDetector(Node):

    def __init__(self):
        super().__init__('detector')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            qos)

    def listener_callback(self, msg):
        self.get_logger().info('Image Frame Id: "%s"' % msg.header.frame_id)

def main(args=None):
    rclpy.init(args=args)
    color_detector = ColorDetector()
    rclpy.spin(color_detector)
    color_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()