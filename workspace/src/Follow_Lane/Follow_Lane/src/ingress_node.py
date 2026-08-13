import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge
import cv2




class ingress_node(Node):
    def __init__(self):
        super().__init__('ingress_node')

        self.get_logger().info('Started ingress_node.')

        self.bridge = CvBridge()

        #--- Subscriber ---
        # Kommt ueber WLAN vom Bot, daher bewusst das komprimierte Topic:
        # /camera/image_raw sind 800*600*3 = 1.44 MB pro Frame, die DDS in ~1000 UDP-Fragmente
        # zerlegt. Faellt davon ein einziges aus, verwirft DDS das ganze Bild -> ueber ein
        # wackliges WLAN kommt dauerhaft gar nichts an. Das JPEG ist ~30x kleiner.
        # qos_profile_sensor_data (BEST_EFFORT) sorgt zusaetzlich dafuer, dass verlorene
        # Pakete den Stream nicht blockieren, statt ihn zum Stillstand zu bringen.
        self.sub_raw_image = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self.cbPubLokalRawImage, qos_profile_sensor_data)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cbPubLokalLIDARScan, qos_profile_sensor_data)

        #--- Publisher ---
        # Bleiben lokal auf dem Laptop, hier ist unkomprimiert unproblematisch.
        self.pub_lokal_raw_image = self.create_publisher(Image, '/lokal/raw_image',1)
        self.pub_lokal_scan = self.create_publisher(LaserScan, '/lokal/scan', 1)

    def cbPubLokalRawImage(self, img):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(img, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Bild konnte nicht dekodiert werden: {e}')
            return

        rotated_image = cv2.rotate(cv_image, cv2.ROTATE_180)
        rotated_msg = self.bridge.cv2_to_imgmsg(rotated_image, encoding='bgr8')
        rotated_msg.header = img.header
        self.pub_lokal_raw_image.publish(rotated_msg)

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
