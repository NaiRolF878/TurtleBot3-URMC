import numpy as np

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from std_msgs.msg import UInt8, Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from follow_lane_msg.msg import LaneMask
from Follow_Lane.src.switch_control import ControlType


class lane_costmap_node(Node):
    """
    Rechnet die von detect_lane auf /detect/lane_mask veroeffentlichten
    Bird's-Eye-Pixelkoordinaten der Spurlinien in Meter-Koordinaten um und
    veroeffentlicht sie als PointCloud2, damit Nav2s obstacle_layer sie als
    virtuelle Waende in die Local Costmap eintragen kann.

    Aktiv nur im Control-Mode ControlType.Obstacle -- im PID-Modus (Lane)
    bleibt der Node still, damit beide Ansaetze konfliktfrei nebeneinander
    existieren koennen.

    Kalibrierung (siehe Parameter-YAML):
    Die Bird's-Eye-Ansicht von detect_lane ist ein quadratisches Bild, dessen
    vier Ecken einer Trapezflaeche auf dem Boden vor dem Roboter entsprechen
    (definiert durch dessen top_left_*/bottom_right_*-Parameter). x_near/x_far
    und y_left/y_right beschreiben, wo die vier Bildkanten dieses Quadrats
    real (in Metern, in frame_id) liegen. Kalibrierung z.B. per Massband:
    Marken auf dem Boden anbringen, /detect/mask_debug/compressed anschauen,
    Pixelposition der Marken ablesen und mit der gemessenen Distanz abgleichen.
    """

    def __init__(self):
        super().__init__('lane_costmap_node')

        self.get_logger().info('Started lane_costmap_node Node.')

        # ---------------------------------
        # Parameter:
        # ---------------------------------
        def frange(lo, hi):
            return ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=lo, to_value=hi, step=0.0)])

        # --- Kalibrierung Pixel -> Meter ---
        # Vorwaertsdistanz (m) der unteren (nahen) bzw. oberen (fernen) Bildkante
        # des Bird's-Eye-Bildes, gemessen ab frame_id.
        self.declare_parameter('x_near', 0.15, frange(-1.0, 5.0))
        self.declare_parameter('x_far',  0.60, frange(-1.0, 5.0))
        # Laterale Position (m) der linken bzw. rechten Bildkante, gemessen ab
        # frame_id. Positiv = links (REP103), z.B. y_left=0.35, y_right=-0.35.
        self.declare_parameter('y_left',  0.35, frange(-5.0, 5.0))
        self.declare_parameter('y_right', -0.35, frange(-5.0, 5.0))

        # --- Sonstige ---
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('output_topic', '/detect/lane_obstacles')
        # Rasterweite beim Ausduennen der Maskenpunkte (1 = jeder Punkt, 2 = jeder
        # zweite in x/y usw.) -- begrenzt Punktzahl und CPU-Last der Costmap.
        self.declare_parameter('pixel_stride', 2,
            ParameterDescriptor(integer_range=[IntegerRange(from_value=1, to_value=20, step=1)]))

        self._param_names = [
            'x_near', 'x_far', 'y_left', 'y_right', 'frame_id', 'pixel_stride',
        ]

        #--- Subscriber ---
        self.sub_lane_mask = self.create_subscription(
            LaneMask, '/detect/lane_mask', self.cbLaneMask, 1)
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cbGetControlMode, 1)

        #--- Publisher ---
        output_topic = self.get_parameter('output_topic').value
        self.pub_lane_obstacles = self.create_publisher(PointCloud2, output_topic, 1)

        self.current_control_mode = ControlType.Lane

    def cbGetControlMode(self, msg):
        try:
            self.current_control_mode = ControlType(msg.data)
        except ValueError:
            self.get_logger().warn(f'Unbekannter Control-Mode empfangen: {msg.data}')

    def cbLaneMask(self, lane_mask):
        if self.current_control_mode != ControlType.Obstacle:
            return

        for name in self._param_names:
            setattr(self, name, self.get_parameter(name).value)

        size = lane_mask.crop_im_size
        if size <= 1:
            return

        stride = max(1, int(self.pixel_stride))

        points = []
        points.extend(self._pixels_to_points(
            lane_mask.yellow_x, lane_mask.yellow_y, size, stride))
        points.extend(self._pixels_to_points(
            lane_mask.white_x, lane_mask.white_y, size, stride))

        header = Header()
        header.stamp = lane_mask.header.stamp
        header.frame_id = self.frame_id

        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.pub_lane_obstacles.publish(cloud)

    def _pixels_to_points(self, xs, ys, size, stride):
        if not xs:
            return []

        xs = np.asarray(xs)
        ys = np.asarray(ys)

        # Grobes, aber schnelles Ausduennen: nur Punkte auf einem stride x
        # stride - Raster behalten, statt jedes Maskenpixel zu verwenden.
        keep = (xs % stride == 0) & (ys % stride == 0)
        xs = xs[keep]
        ys = ys[keep]
        if xs.size == 0:
            return []

        # row=0 (oben, fern) -> x_far, row=size-1 (unten, nah) -> x_near
        forward = self.x_far + (ys.astype(np.float64) / (size - 1)) * (self.x_near - self.x_far)
        # col=0 (links) -> y_left, col=size-1 (rechts) -> y_right
        lateral = self.y_left + (xs.astype(np.float64) / (size - 1)) * (self.y_right - self.y_left)
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
