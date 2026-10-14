import os

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from follow_lane_msg.msg import LaneMask


class lane_costmap_node(Node):
    """Spurlinien-Pixel von /detect/lane_mask als PointCloud2 fuer Nav2s
    obstacle_layer -- virtuelle Waende in der Local Costmap.

    Laeuft unabhaengig vom Control-Mode durchgehend: der Node ist reine
    Wahrnehmung und sendet keine Fahrbefehle. Gesperrt wird in dieser
    Architektur nur, wer faehrt (control_driving, nav2_goal_publisher).
    Durchgehender Betrieb heisst, dass der lane_layer beim Umschalten auf
    Obstacle schon einen Korridor enthaelt, statt erst dann leer anzufangen.

    Pixel -> Meter ueber die vermessene Bodenhomographie H_image_to_ground aus
    Parameter/camera_info/<ground_homography_file> (erzeugt mit
    scripts/tb/ground-calib.py). Sie gilt nur fuer das entzerrte Vollbild in genau
    der dort eingetragenen Aufloesung; bei abweichender Bildgroesse wird nichts
    gesendet, weil die Meter sonst falsch waeren.

    Ergebnis ist X vorwaerts ab Radachse, Y nach links, in Metern am Boden. Das
    ist genau base_footprint: in der TurtleBot3-URDF (burger) sitzen
    wheel_left/right_joint bei x=0 relativ zu base_link, und base_link liegt
    0.010 m ueber base_footprint; die Achse hat damit 0.010+0.023 = 0.033 m Hoehe
    = Radradius. base_footprint liegt also am Boden direkt unter der Radachse,
    die Punkte gehen ohne weitere Transformation in frame_id=base_footprint.
    """

    def __init__(self):
        super().__init__('lane_costmap_node')

        self.get_logger().info('Started lane_costmap_node Node.')

        def frange(lo, hi):
            return ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=lo, to_value=hi, step=0.0)])

        # Dateiname in Parameter/camera_info/, wird nur beim Start gelesen.
        self.declare_parameter('ground_homography_file', 'ground_homography_800x600.yaml')
        # Gueltigkeitsbereich der Homographie in X (vorwaerts ab Radachse). Punkte
        # ausserhalb werden verworfen. Naeher als ground_x_min wird sie unzuverlaessig;
        # ground_x_max: die Marker beim Fit reichten nur bis 0.70 m, darueber hinaus
        # wird extrapoliert und die Fehler wachsen stark.
        self.declare_parameter('ground_x_min', 0.15, frange(0.0, 5.0))
        self.declare_parameter('ground_x_max', 0.80, frange(0.0, 5.0))

        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('output_topic', '/detect/lane_obstacles')
        self.declare_parameter('pixel_stride', 2,
            ParameterDescriptor(integer_range=[IntegerRange(from_value=1, to_value=20, step=1)]))

        self._param_names = [
            'ground_x_min', 'ground_x_max', 'frame_id', 'pixel_stride',
        ]

        self.sub_lane_mask = self.create_subscription(
            LaneMask, '/detect/lane_mask', self.cbLaneMask, 1)

        output_topic = self.get_parameter('output_topic').value
        self.pub_lane_obstacles = self.create_publisher(PointCloud2, output_topic, 1)

        self._H = None
        self._H_size = None
        self._size_error_logged = False
        self._load_homography(self.get_parameter('ground_homography_file').value)

    def _load_homography(self, filename):
        path = os.path.join(get_package_share_directory('Follow_Lane'),
                            'Parameter', 'camera_info', filename)
        try:
            with open(path) as f:
                d = yaml.safe_load(f)
            H = np.array(d['H_image_to_ground'], dtype=np.float64).reshape(3, 3)
            width, height = int(d['image_width']), int(d['image_height'])
        except Exception as e:
            self.get_logger().error(
                f'Bodenhomographie {path} nicht ladbar ({e}) -- es werden keine Spurwaende gesendet.')
            return

        # Pixel oberhalb des Horizonts treffen den Boden nicht, w hat dort das andere
        # Vorzeichen. Welches Vorzeichen "Boden" ist, haengt von der Skalierung des Fits
        # ab -- daher an der unteren Bildmitte bestimmen, die sicher den Boden zeigt.
        self._ground_w_sign = np.sign((H @ [width / 2.0, height - 1.0, 1.0])[2])
        self._H = H
        self._H_size = (width, height)
        self.get_logger().info(
            f'Bodenhomographie {filename} geladen ({width}x{height}, '
            f'Kreuzvalidierung {d.get("crossval_rms_mm", float("nan")):.1f} mm RMS).')

    def cbLaneMask(self, lane_mask):
        if self._H is None:
            return

        if (lane_mask.image_width, lane_mask.image_height) != self._H_size:
            if not self._size_error_logged:
                self.get_logger().error(
                    f'LaneMask kommt mit {lane_mask.image_width}x{lane_mask.image_height}, '
                    f'die Bodenhomographie gilt fuer {self._H_size[0]}x{self._H_size[1]} '
                    '-- es werden keine Spurwaende gesendet.')
                self._size_error_logged = True
            return

        for name in self._param_names:
            setattr(self, name, self.get_parameter(name).value)

        stride = max(1, int(self.pixel_stride))

        points = []
        points.extend(self._pixels_to_points(
            lane_mask.yellow_x, lane_mask.yellow_y, stride))
        points.extend(self._pixels_to_points(
            lane_mask.white_x, lane_mask.white_y, stride))

        header = Header()
        header.stamp = lane_mask.header.stamp
        header.frame_id = self.frame_id

        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.pub_lane_obstacles.publish(cloud)

    def _pixels_to_points(self, xs, ys, stride):
        if not xs:
            return []

        xs = np.asarray(xs)
        ys = np.asarray(ys)

        keep = (xs % stride == 0) & (ys % stride == 0)
        xs = xs[keep]
        ys = ys[keep]
        if xs.size == 0:
            return []

        # Homogen: [u, v, 1] -> [X, Y, w], dann durch w teilen
        uv1 = np.vstack([xs, ys, np.ones_like(xs)]).astype(np.float64)
        ground = self._H @ uv1
        w = ground[2]
        on_ground = w * self._ground_w_sign > 1e-9
        forward = ground[0, on_ground] / w[on_ground]
        lateral = ground[1, on_ground] / w[on_ground]

        in_range = (forward >= self.ground_x_min) & (forward <= self.ground_x_max)
        forward = forward[in_range]
        lateral = lateral[in_range]
        elevation = np.zeros_like(forward)

        return list(zip(forward.tolist(), lateral.tolist(), elevation.tolist()))


def main(args=None):
    rclpy.init(args=args)
    node = lane_costmap_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy raeumt den Context im eigenen SIGINT-Handler ab
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
