import numpy as np

import rclpy
from rclpy.node import Node
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

    x_near/x_far/y_left/y_right sagen, wo die vier Kanten des Bird's-Eye-Bildes
    real liegen (Meter, in frame_id). Per Massband kalibrieren.
    """

    def __init__(self):
        super().__init__('lane_costmap_node')

        self.get_logger().info('Started lane_costmap_node Node.')

        def frange(lo, hi):
            return ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=lo, to_value=hi, step=0.0)])

        # Vorwaertsdistanz der nahen/fernen Bildkante
        self.declare_parameter('x_near', 0.15, frange(-1.0, 5.0))
        self.declare_parameter('x_far',  0.60, frange(-1.0, 5.0))
        # Laterale Lage der linken/rechten Bildkante, positiv = links (REP103)
        self.declare_parameter('y_left',  0.35, frange(-5.0, 5.0))
        self.declare_parameter('y_right', -0.35, frange(-5.0, 5.0))

        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('output_topic', '/detect/lane_obstacles')
        self.declare_parameter('pixel_stride', 2,
            ParameterDescriptor(integer_range=[IntegerRange(from_value=1, to_value=20, step=1)]))

        self._param_names = [
            'x_near', 'x_far', 'y_left', 'y_right', 'frame_id', 'pixel_stride',
        ]

        self.sub_lane_mask = self.create_subscription(
            LaneMask, '/detect/lane_mask', self.cbLaneMask, 1)

        output_topic = self.get_parameter('output_topic').value
        self.pub_lane_obstacles = self.create_publisher(PointCloud2, output_topic, 1)

    def cbLaneMask(self, lane_mask):
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

        keep = (xs % stride == 0) & (ys % stride == 0)
        xs = xs[keep]
        ys = ys[keep]
        if xs.size == 0:
            return []

        # Zeile 0 = fern, letzte Zeile = nah; Spalte 0 = links
        forward = self.x_far + (ys.astype(np.float64) / (size - 1)) * (self.x_near - self.x_far)
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
        # rclpy raeumt den Context im eigenen SIGINT-Handler ab
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
