
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np
import math as m
import cv2


class avoid_obstacle(Node):
    def __init__(self):
        super().__init__('avoid_obstacle')

        self.bridge = CvBridge()

        # Subscriber
        self.sub_lokal_raw_scan = self.create_subscription(LaserScan, '/lokal/scan', self.cb_detect_obstacle, 1)

        #Publisher
        self.pub_obstacle_detected = self.create_publisher(Bool, '/detect/obstacle',1)
        self.pub_lidar_image = self.create_publisher(Image,'/detect/lidarImage',1)

    def cb_detect_obstacle(self,scan):
        #scan = LaserScan()
        if scan.range_min <= 0.2:
            msg = Bool()
            msg.data = True
            self.pub_obstacle_detected.publish(msg)
        else:
            return

        cartesian = self.convert_lidar_cartesian(scan)
        self.show_cartesian_obstacles(cartesian)
        return

        


    def convert_lidar_cartesian(self,scan):
        cartesian_points = []
        #angle= (180/np.pi) /scan.angle_increment
        angle =scan.angle_increment
        current_point=0
        for current_range in scan.ranges:
            current_angle = angle * current_point
            current_point +=1
            # inf/nan stehen fuer "kein Echo" -- int() wuerde daran mit OverflowError sterben
            if not m.isfinite(current_range):
                continue
            x = current_range * m.cos(current_angle)
            y = current_range * m.sin(current_angle)
            cartesian_points.append((x,y))

        return cartesian_points

    def show_cartesian_obstacles(self, cartesian_points):
        map_size = 700
        center = map_size // 2
        meter_in_pixel= 100

        # x und y einzeln skalieren: (x,y)*100 wuerde das Tupel vervielfachen statt rechnen
        true_points = []
        for x, y in cartesian_points:
            true_points.append((int(x*meter_in_pixel)+center, int(y*meter_in_pixel)+center))

        Lidar_map = np.zeros((map_size, map_size, 3), dtype=np.uint8)
        for point in true_points:
            cv2.circle(Lidar_map,point,3, (255, 255, 255), -1, cv2.LINE_AA)

        self.pub_lidar_image.publish(self.bridge.cv2_to_imgmsg(Lidar_map, encoding='bgr8'))


def main(args=None):
    # SIGINT selbst behandeln (KeyboardInterrupt), damit der Context beim Aufraeumen noch
    # gilt -- sonst bei Strg+C ExternalShutdownException-/"context is invalid"-Tracebacks.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = avoid_obstacle()
    try:
        # spin_once mit Timeout statt spin(): spin() wartet in C auf die naechste Nachricht,
        # Strg+C kaeme ohne Daten nie an (Knoten haengt beim Beenden).
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy raeumt den Context im eigenen SIGINT-Handler ab -- sonst wirft der zweite Aufruf
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

        


