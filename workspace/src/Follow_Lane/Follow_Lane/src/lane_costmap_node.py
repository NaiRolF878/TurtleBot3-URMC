import math
import os
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from std_msgs.msg import Header, UInt8
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.srv import ClearEntireCostmap
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from follow_lane_msg.msg import LaneMask
from Follow_Lane.src import bev_geometry
from Follow_Lane.src.switch_control import ControlType


def _bin_medians(pts, x_min, x_max, bin_size):
    """Median-Punkt der Linienpixel pro Streifen [x, x+bin_size) -> (x, y)-Arrays.

    Der Median liegt auf der Mitte des Linienstreifens. Eine Kante (Min/Max)
    waere bei schraeger Linie verzerrt, weil die Linie innerhalb eines Streifens
    seitlich wandert; einzelne Fehlpixel verschieben den Median kaum.
    """
    xs, ys = [], []
    edges = np.arange(x_min, x_max + 1e-9, bin_size)
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (pts[0] >= lo) & (pts[0] < hi)
        if m.any():
            xs.append(np.median(pts[0][m]))
            ys.append(np.median(pts[1][m]))
    return np.array(xs), np.array(ys)


def _fit(xs, ys):
    """Polynom Y(X) durch die Linienpunkte: 2. Grad ab 4 Punkten, sonst Gerade."""
    if len(xs) < 2:
        return None
    return np.polyfit(xs, ys, 2 if len(xs) >= 4 else 1)


def _slope(coeff, x):
    return 0.0 if coeff is None else float(np.polyval(np.polyder(coeff), x))


def lane_center_points(yellow, white, x_min, x_max, bin_size, half_width):
    """Punkte auf der Spurmitte, in base_footprint.

    yellow/white: (forward, lateral)-Arrays. Gelb liegt links, Weiss rechts.
    Jede Linie wird fuer sich durch ein Polynom angenaehert (wie bei ROBOTIS
    turtlebot3_autorace), dann wird jeder ihrer Punkte senkrecht zur Linie um
    die halbe Spurbreite zur Mitte versetzt. Senkrecht, nicht entlang Y: in
    Kurven waere der Abstand entlang Y groesser als die echte Breite.

    Die Spurbreite wird gemessen, nicht vorgegeben: von Linienmitte zu
    Linienmitte, ueberall dort, wo beide Linien zu sehen sind. Die Linienbreite
    faellt dabei heraus. Sieht dieses Bild nur eine Linie (Luecke in Weiss,
    enge Kurve, Hindernis verdeckt), gilt half_width aus den vorigen Bildern;
    ist die noch unbekannt (None), gibt es keine Mittelpunkte.

    Weiss links der gelben Linie gehoert nicht zu dieser Spur und wird ignoriert.
    Gibt (x, y, breiten) zurueck; breiten sind die in diesem Bild gemessenen.
    """
    empty = (np.empty(0), np.empty(0), [])

    yx, yy = _bin_medians(yellow, x_min, x_max, bin_size)
    y_fit = _fit(yx, yy)
    if y_fit is not None and white[0].size:
        keep = white[1] < np.polyval(y_fit, white[0])
        white = (white[0][keep], white[1][keep])
    wx, wy = _bin_medians(white, x_min, x_max, bin_size)
    w_fit = _fit(wx, wy)

    widths = []
    if y_fit is not None and w_fit is not None:
        for x in np.arange(max(yx.min(), wx.min()), min(yx.max(), wx.max()) + 1e-9, bin_size):
            k = (_slope(y_fit, x) + _slope(w_fit, x)) / 2.0
            width = (np.polyval(y_fit, x) - np.polyval(w_fit, x)) / np.hypot(1.0, k)
            if width > 0:
                widths.append(float(width))

    hw = float(np.median(widths)) / 2.0 if widths else half_width
    if hw is None:
        return empty

    cx, cy = [], []
    for x, y in zip(yx, yy):          # Normale nach rechts: (k, -1) / |(1, k)|
        k = _slope(y_fit, x)
        n = np.hypot(1.0, k)
        cx.append(x + k * hw / n)
        cy.append(y - hw / n)
    for x, y in zip(wx, wy):          # Normale nach links: (-k, 1) / |(1, k)|
        k = _slope(w_fit, x)
        n = np.hypot(1.0, k)
        cx.append(x - k * hw / n)
        cy.append(y + hw / n)
    return np.array(cx), np.array(cy), widths


def fit_center_line(xs, ys, step):
    """Glatte Mittellinie durch die Mittelpunkte, in base_footprint.

    Polynom zweiten Grades Y(X) ab 4 Punkten, darunter eine Gerade. Abgetastet
    wird nur zwischen dem naechsten und dem fernsten Mittelpunkt -- keine
    Extrapolation. Gibt (x, y, yaw) zurueck oder None, wenn weniger als zwei
    Posen herauskaemen.
    """
    if len(xs) < 2:
        return None
    deg = 2 if len(xs) >= 4 else 1
    coeff = np.polyfit(xs, ys, deg)
    px = np.arange(xs.min(), xs.max() + 1e-9, step)
    if px.size < 2:
        return None
    py = np.polyval(coeff, px)
    yaw = np.arctan(np.polyval(np.polyder(coeff), px))
    return px, py, yaw


class lane_costmap_node(Node):
    """Spurlinien-Pixel von /detect/lane_mask als PointCloud2 fuer Nav2s
    obstacle_layer -- virtuelle Waende in der Local Costmap, zwischen denen MPPI
    im Obstacle-Modus ausweicht.

    Sendet in jedem Modus ausser Tunnel: so enthaelt der lane_layer beim
    Umschalten auf Obstacle schon einen Korridor, statt erst dann leer
    anzufangen. Im Tunnel ist die Kamera laut Reglement verboten -- dort sendet
    der Node nichts und leert beim Eintritt die Local Costmap, damit auch keine
    vorher gemerkten Spurwaende mehr darin liegen.

    Die Korridorbreite muss nirgends eingestellt werden: MPPI prueft Kollisionen
    mit dem echten Footprint des Burger gegen die Linienzellen (siehe
    nav2_params.yaml, local_costmap.footprint).

    Zusaetzlich die Spurmitte als nav_msgs/Path (center_path_topic), im
    odom-Frame zum Zeitpunkt des Bildes: nav2_goal_publisher gibt sie MPPI im
    Obstacle-Modus statt einer Geraden, damit der Bot auch in Kurven der Spur
    folgt. Im odom-Frame bleibt der Pfad liegen, wenn der Bot beim Ausweichen
    schraeg steht und die Linien kurz nicht sieht.

    Die Linienpixel kommen aus der metrischen Vogelperspektive /lokal/bev, die
    ingress_node aus der vermessenen Extrinsik erzeugt. Pixel -> Meter ist dort
    linear und steht in Parameter/bev.yaml (gleiche Datei wie fuer ingress_node):
        X = x_max - (v + 0.5) * res,   Y = y_half - (u + 0.5) * res
    Kommt die LaneMask in einer anderen Groesse als bev.yaml beschreibt, wird
    nichts gesendet, weil die Meter sonst falsch waeren.

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

        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('output_topic', '/detect/lane_obstacles')
        self.declare_parameter('pixel_stride', 2,
            ParameterDescriptor(integer_range=[IntegerRange(from_value=1, to_value=20, step=1)]))

        # --- Spurmitte ---
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('center_path_topic', '/lane/center_path')
        # Tiefe eines Streifens; gleich der Zellgroesse der local_costmap, feiner
        # kann MPPI ohnehin nicht unterscheiden.
        self.declare_parameter('center_bin_size', 0.05, frange(0.01, 0.5))
        # Mindestzahl Mittelpunkte, sonst wird kein Pfad gesendet.
        self.declare_parameter('center_min_points', 3,
            ParameterDescriptor(integer_range=[IntegerRange(from_value=2, to_value=50, step=1)]))
        # Ueber so viele Bilder wird die gemessene Spurbreite (Linienmitte zu
        # Linienmitte) gemittelt (Median). Gilt, wenn ein Bild nur eine Linie sieht.
        # Kurz genug, dass der Wechsel von der 20-cm-Spur in den breiteren
        # Hindernisbereich nach wenigen Bildern nachgezogen ist.
        self.declare_parameter('center_width_history', 20,
            ParameterDescriptor(integer_range=[IntegerRange(from_value=1, to_value=500, step=1)]))

        self._param_names = [
            'frame_id', 'pixel_stride',
            'odom_frame', 'center_bin_size', 'center_min_points',
        ]

        self.sub_lane_mask = self.create_subscription(
            LaneMask, '/detect/lane_mask', self.cbLaneMask, 1)
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cbGetControlMode, 1)

        output_topic = self.get_parameter('output_topic').value
        self.pub_lane_obstacles = self.create_publisher(PointCloud2, output_topic, 1)
        self.pub_center_path = self.create_publisher(
            Path, self.get_parameter('center_path_topic').value, 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._widths = deque(maxlen=self.get_parameter('center_width_history').value)

        self.clear_local_costmap = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

        self.current_control_mode = ControlType.Lane

        self._bev = None
        self._size_error_logged = False
        self._load_bev()

    def _load_bev(self):
        param_dir = os.path.join(get_package_share_directory('Follow_Lane'), 'Parameter')
        try:
            self._bev = bev_geometry.load_bev_spec(param_dir)
        except Exception as e:
            self.get_logger().error(
                f'{bev_geometry.BEV_FILE} nicht ladbar ({e}) -- es werden keine Spurwaende gesendet.')
            return
        b = self._bev
        self.get_logger().info(
            f'BEV aus {bev_geometry.BEV_FILE}: {b["width"]}x{b["height"]} px, '
            f'{b["res"] * 1000:.2f} mm/px, X {b["x_min"]:.2f}-{b["x_max"]:.2f} m.')

    def cbGetControlMode(self, msg):
        try:
            new_mode = ControlType(msg.data)
        except ValueError:
            self.get_logger().warn(f'Unbekannter Control-Mode empfangen: {msg.data}')
            return

        entering_tunnel = (new_mode == ControlType.Tunnel
                           and self.current_control_mode != ControlType.Tunnel)
        self.current_control_mode = new_mode
        if entering_tunnel:
            self._clear_local_costmap()

    def _clear_local_costmap(self):
        # Leert alle Layer; LiDAR und Eingangssperre fuellen sich im naechsten Zyklus neu.
        if not self.clear_local_costmap.service_is_ready():
            self.get_logger().error(
                'local_costmap-Clear-Service nicht erreichbar -- alte Spurwaende bleiben '
                'in der Karte. Laeuft controller_server?')
            return
        self.clear_local_costmap.call_async(ClearEntireCostmap.Request())

    def cbLaneMask(self, lane_mask):
        if self.current_control_mode == ControlType.Tunnel:
            return
        if self._bev is None:
            return

        size = (self._bev['width'], self._bev['height'])
        if (lane_mask.image_width, lane_mask.image_height) != size:
            if not self._size_error_logged:
                self.get_logger().error(
                    f'LaneMask kommt mit {lane_mask.image_width}x{lane_mask.image_height}, '
                    f'{bev_geometry.BEV_FILE} beschreibt {size[0]}x{size[1]} '
                    '-- es werden keine Spurwaende gesendet.')
                self._size_error_logged = True
            return

        for name in self._param_names:
            setattr(self, name, self.get_parameter(name).value)

        stride = max(1, int(self.pixel_stride))

        yellow = self._pixels_to_ground(lane_mask.yellow_x, lane_mask.yellow_y, stride)
        white = self._pixels_to_ground(lane_mask.white_x, lane_mask.white_y, stride)

        header = Header()
        header.stamp = lane_mask.header.stamp
        header.frame_id = self.frame_id

        forward = np.concatenate([yellow[0], white[0]])
        lateral = np.concatenate([yellow[1], white[1]])
        points = list(zip(forward.tolist(), lateral.tolist(), [0.0] * forward.size))
        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.pub_lane_obstacles.publish(cloud)

        self._publish_center_path(yellow, white, lane_mask.header.stamp)

    def _publish_center_path(self, yellow, white, stamp):
        half_width = float(np.median(self._widths)) / 2.0 if self._widths else None
        xs, ys, widths = lane_center_points(
            yellow, white, self._bev['x_min'], self._bev['x_max'],
            self.center_bin_size, half_width)
        if widths:
            self._widths.append(float(np.median(widths)))
        if len(xs) < self.center_min_points:
            return

        line = fit_center_line(xs, ys, self.center_bin_size)
        if line is None:
            return

        # Pose des Bots zum Bildzeitpunkt, nicht jetzt -- sonst laege der Pfad um
        # die seitdem gefahrene Strecke versetzt. Fehlt die TF dafuer noch, faellt
        # dieses Bild fuer den Pfad aus; die Spurwaende sind schon gesendet.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.frame_id, Time.from_msg(stamp))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'Keine TF {self.odom_frame} <- {self.frame_id} zum Bildzeitpunkt: {e}',
                throttle_duration_sec=2.0)
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(robot_yaw), math.sin(robot_yaw)

        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.odom_frame
        for px, py, pyaw in zip(*line):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = t.x + c * px - s * py
            pose.pose.position.y = t.y + s * px + c * py
            yaw = robot_yaw + pyaw
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        self.pub_center_path.publish(path)

    def _pixels_to_ground(self, xs, ys, stride):
        """Linienpixel -> (forward, lateral) in Metern, base_footprint."""
        empty = (np.empty(0), np.empty(0))
        if not xs:
            return empty

        xs = np.asarray(xs)
        ys = np.asarray(ys)

        keep = (xs % stride == 0) & (ys % stride == 0)
        xs = xs[keep]
        ys = ys[keep]
        if xs.size == 0:
            return empty

        # Linear, Pixelmitte: siehe bev.yaml
        b = self._bev
        forward = b['x_max'] - (ys + 0.5) * b['res']
        lateral = b['y_half'] - (xs + 0.5) * b['res']
        return forward, lateral


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
