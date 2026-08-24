import math
from enum import Enum

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from std_msgs.msg import UInt8
from sensor_msgs.msg import LaserScan

from follow_lane_msg.msg import LaneMask


class ControlType(Enum):
    Lane = 1
    Obstacle = 2
    Stop = 3
    Intersection = 4
    Align = 5
    Tunnel = 6


class switch_control(Node):
    """
    Zentrale Zustandsmaschine fuer /control/mode. Zwei automatische Uebergaenge:

    Lane <-> Obstacle: rein LiDAR-basiert (/scan). Sobald etwas in einem
    Pruefkegel vor dem Roboter naeher als obstacle_trigger_distance liegt,
    wird von Lane zu Obstacle gewechselt; erst wenn dieser Kegel fuer
    obstacle_clear_debounce Sekunden durchgehend frei ist, geht es zurueck.
    Nur relevant, solange NICHT Tunnel aktiv ist -- der Tunnel-Modus weicht
    ueber dieselbe Local Costmap ohnehin die ganze Zeit aus.

    -> Tunnel -> zurueck: Eintritt per Schild-Erkennung auf /detect/sign
    (Publisher dafuer existiert aktuell noch nicht -- die Schild-Detektor-
    Node wird erst gebaut, sobald klar ist, wie die Schilder aussehen; bis
    dahin manuell testbar per
      ros2 topic pub /detect/sign std_msgs/msg/UInt8 "{data: 6}"
    ). Austritt erst, wenn LiDAR wieder "offenen Raum" sieht UND die Kamera
    wieder Spurpixel liefert (/detect/lane_mask), beides gleichzeitig fuer
    tunnel_exit_debounce Sekunden durchgehend. Beim Eintritt wird der vorherige
    Modus gemerkt und beim Austritt wiederhergestellt (nicht hart auf Lane).
    """

    def __init__(self):
        super().__init__('switch_control')

        self.get_logger().info('Started switch_control Node.')

        # ---------------------------------
        # Parameter:
        # ---------------------------------
        def frange(lo, hi):
            return ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=lo, to_value=hi, step=0.0)])

        # --- Obstacle-Erkennung (Lane <-> Obstacle) ---
        self.declare_parameter('obstacle_check_angle_deg', 30.0, frange(1.0, 90.0))
        self.declare_parameter('obstacle_trigger_distance', 0.4, frange(0.05, 3.0))
        self.declare_parameter('obstacle_clear_debounce', 1.0, frange(0.0, 10.0))

        # --- Tunnel-Austritt (Tunnel -> vorheriger Modus) ---
        self.declare_parameter('tunnel_exit_open_distance', 1.5, frange(0.1, 10.0))
        self.declare_parameter('tunnel_exit_open_ratio', 0.8, frange(0.0, 1.0))
        self.declare_parameter('tunnel_exit_min_lane_pixels', 20,
            ParameterDescriptor(integer_range=[IntegerRange(from_value=0, to_value=5000, step=1)]))
        self.declare_parameter('tunnel_exit_debounce', 1.0, frange(0.0, 10.0))

        self._param_names = [
            'obstacle_check_angle_deg', 'obstacle_trigger_distance', 'obstacle_clear_debounce',
            'tunnel_exit_open_distance', 'tunnel_exit_open_ratio',
            'tunnel_exit_min_lane_pixels', 'tunnel_exit_debounce',
        ]

        #--- Subscriber ---
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cbScan, qos_profile_sensor_data)
        self.sub_lane_mask = self.create_subscription(
            LaneMask, '/detect/lane_mask', self.cbLaneMask, 1)
        # Noch ohne Publisher (siehe Docstring) -- Interface steht schon, damit die
        # kuenftige Schild-Detektor-Node nur noch andocken muss.
        self.sub_sign = self.create_subscription(UInt8, '/detect/sign', self.cbSign, 1)

        #--- Publisher ---
        self.pub_control = self.create_publisher(UInt8, '/control/mode', 1)

        # Ohne Timer wurde der Mode nie gesendet -> control_driving hat nie einen
        # Mode empfangen. Zyklisch senden, damit auch spaeter startende Nodes ihn bekommen.
        self.timer = self.create_timer(0.5, self.cbPubControlMode)

        #--- Zustand ---
        self.current_control_mode = ControlType.Lane
        self._mode_before_tunnel = None
        self._obstacle_clear_since = None
        self._tunnel_exit_since = None
        self._lane_visible = False

    def _read_params(self):
        for name in self._param_names:
            setattr(self, name, self.get_parameter(name).value)

    def cbPubControlMode(self):
        msg = UInt8()
        msg.data = self.current_control_mode.value
        self.pub_control.publish(msg)

    def cbSign(self, msg):
        try:
            sign = ControlType(msg.data)
        except ValueError:
            self.get_logger().warn(f'Unbekanntes Schild-Signal: {msg.data}')
            return

        if sign == ControlType.Tunnel:
            if self.current_control_mode != ControlType.Tunnel:
                self._mode_before_tunnel = self.current_control_mode
                self.current_control_mode = ControlType.Tunnel
                self._tunnel_exit_since = None
                self.get_logger().info('Tunnel-Schild erkannt -> wechsle zu Tunnel-Modus.')
        else:
            self.get_logger().debug(
                f'Schild {sign.name} erkannt, dafuer ist noch keine Aktion definiert.')

    def cbLaneMask(self, lane_mask):
        self._read_params()
        total_pixels = len(lane_mask.yellow_x) + len(lane_mask.white_x)
        self._lane_visible = total_pixels >= self.tunnel_exit_min_lane_pixels

    def cbScan(self, scan):
        self._read_params()

        if self.current_control_mode == ControlType.Tunnel:
            self._update_tunnel_exit(scan)
        elif self.current_control_mode in (ControlType.Lane, ControlType.Obstacle):
            self._update_obstacle_mode(scan)
        # Andere Modi (Stop/Intersection/Align): hier (noch) keine automatische Logik.

    def _update_obstacle_mode(self, scan):
        obstacle_present = self._forward_obstacle_present(scan)

        if obstacle_present:
            self._obstacle_clear_since = None
            if self.current_control_mode == ControlType.Lane:
                self.current_control_mode = ControlType.Obstacle
                self.get_logger().info('Hindernis in der Fahrbahn erkannt -> wechsle zu Obstacle-Modus.')
        elif self.current_control_mode == ControlType.Obstacle:
            now = self.get_clock().now()
            if self._obstacle_clear_since is None:
                self._obstacle_clear_since = now
            elif (now - self._obstacle_clear_since).nanoseconds / 1e9 >= self.obstacle_clear_debounce:
                self.current_control_mode = ControlType.Lane
                self._obstacle_clear_since = None
                self.get_logger().info('Kein Hindernis mehr erkannt -> zurueck zu Lane-Modus.')

    def _forward_obstacle_present(self, scan):
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        if ranges.size == 0:
            return False

        angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
        # Auf [-pi, pi] normalisieren, damit "geradeaus" (0 rad) unabhaengig von
        # Start-/Endwinkel des Scans korrekt geprueft wird.
        angles = np.arctan2(np.sin(angles), np.cos(angles))

        half_angle = math.radians(self.obstacle_check_angle_deg)
        in_cone = np.abs(angles) <= half_angle
        valid = np.isfinite(ranges) & (ranges > scan.range_min)
        close = ranges < self.obstacle_trigger_distance

        return bool(np.any(in_cone & valid & close))

    def _update_tunnel_exit(self, scan):
        scan_open = self._is_open_space(scan)

        if scan_open and self._lane_visible:
            now = self.get_clock().now()
            if self._tunnel_exit_since is None:
                self._tunnel_exit_since = now
            elif (now - self._tunnel_exit_since).nanoseconds / 1e9 >= self.tunnel_exit_debounce:
                self._leave_tunnel()
        else:
            self._tunnel_exit_since = None

    def _is_open_space(self, scan):
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        valid = np.isfinite(ranges) & (ranges > scan.range_min)
        if not np.any(valid):
            return False

        open_mask = ranges[valid] > self.tunnel_exit_open_distance
        open_ratio = np.count_nonzero(open_mask) / open_mask.size
        return open_ratio >= self.tunnel_exit_open_ratio

    def _leave_tunnel(self):
        restored = self._mode_before_tunnel or ControlType.Lane
        self.get_logger().info(f'Tunnel verlassen -> zurueck zu {restored.name}.')
        self.current_control_mode = restored
        self._mode_before_tunnel = None
        self._tunnel_exit_since = None


def main(args=None):
    rclpy.init(args=args)
    node = switch_control()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:

        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
