import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge




class ingress_node(Node):
    def __init__(self):
        super().__init__('ingress_node')

        self.get_logger().info('Started ingress_node.')

        self.bridge = CvBridge()

        #--- Subscriber ---
        self.sub_raw_image = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self.cbPubLokalRawImage, qos_profile_sensor_data)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cbPubLokalLIDARScan, qos_profile_sensor_data)

        #--- Publisher ---
        self.pub_lokal_raw_image = self.create_publisher(Image, '/lokal/raw_image',1)
        self.pub_lokal_scan = self.create_publisher(LaserScan, '/lokal/scan', 1)

    def cbPubLokalRawImage(self, img):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(img, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Bild konnte nicht dekodiert werden: {e}')
            return

        out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        out_msg.header = img.header
        self.pub_lokal_raw_image.publish(out_msg)

    def cbPubLokalLIDARScan(self, scan):
        self.pub_lokal_scan.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = ingress_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
