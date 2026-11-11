"""Durchfahrt einer Engstelle im Tunnel als Teil von switch_control; der Modus bleibt Tunnel.

Ein:  /tunnel/passage_line (Path, odom: Start und Ende der Geraden, von tunnel_explorer_node),
      /control/maneuver_status (UInt32), TF
Aus:  /control/maneuver (Maneuver), /tunnel/passage (Bool, True solange sie laeuft)
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, UInt32
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from follow_lane_msg.msg import Maneuver
from Follow_Lane.src.common import ControlType, pose2d_from_transform

ODOM_FRAME = 'odom'
ROBOT_FRAME = 'base_footprint'
TICK_PERIOD_S = 0.1
PASSAGE_VEL = 0.05
STEP_TIMEOUT_S = 20.0


class TunnelPassage:
    """Ablauf: Pfad zum Start der Geraden (control_driving dreht dafuer auf der Stelle und faehrt
    vor oder zurueck), auf der Stelle in Richtung Ende drehen, langsam geradeaus mit gehaltenem
    Kurs bis zum Ende. Danach /tunnel/passage False, der Explorer plant
    weiter. Dauert ein Schritt laenger als STEP_TIMEOUT_S, haelt eine leere Grundbewegung den
    Bot an; verlaesst er den Tunnelmodus, verwirft control_driving die Bewegung selbst."""

    def __init__(self, node):
        self.node = node

        # --- Subscriber ---
        node.create_subscription(Path, '/tunnel/passage_line', self.cb_line, 1)
        node.create_subscription(UInt32, '/control/maneuver_status', self.cb_maneuver_status, 1)

        # --- Publisher ---
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_passage = node.create_publisher(Bool, '/tunnel/passage', latched)
        self.pub_maneuver = node.create_publisher(Maneuver, '/control/maneuver', 1)

        # --- Timer ---
        node.create_timer(TICK_PERIOD_S, self.cb_tick)

        # --- Zustand ---
        self._queue = []                # noch abzufahrende Grundbewegungen
        self._since = 0.0
        # Andere Nummern als ein frisch gestarteter Ablauf: control_driving startet eine id,
        # die er zuletzt beendet hat, nie erneut.
        self._maneuver_id = int(time.time()) % 1_000_000 * 1000
        self.pub_passage.publish(Bool(data=False))

    @property
    def active(self):
        return bool(self._queue)

    def cancel(self):
        if self.active:
            self.node.get_logger().info('Durchfahrt abgebrochen: Tunnelmodus verlassen.')
            self._finish()

    # --- Callbacks ---

    def cb_line(self, msg):
        if self.active or len(msg.poses) != 2:
            return
        log = self.node.get_logger()
        pose = self._robot_pose() if self.node.control_mode == ControlType.Tunnel else None
        if pose is None:
            log.warn('Durchfahrt abgelehnt: nicht im Tunnel oder keine Roboterpose.')
            self.pub_passage.publish(Bool(data=False))
            return
        start, end = ((p.pose.position.x, p.pose.position.y) for p in msg.poses)
        yaw = math.atan2(end[1] - start[1], end[0] - start[0])
        length = math.dist(start, end)
        ahead = (start[0] - pose[0]) * math.cos(pose[2]) + (start[1] - pose[1]) * math.sin(pose[2])
        self._queue = [self._path_to(start, reverse=ahead < 0.0), self._rotate(yaw),
                       self._straight(length, PASSAGE_VEL)]
        log.info(f'Durchfahrt: {"zurueck" if ahead < 0.0 else "vor"} zum Start '
                 f'x={start[0]:.2f} y={start[1]:.2f}, drehen auf {math.degrees(yaw):.0f} Grad, '
                 f'{length:.2f} m geradeaus.')
        self._since = time.monotonic()
        self.pub_passage.publish(Bool(data=True))
        self._publish_maneuver()

    def cb_maneuver_status(self, msg):
        if not self._queue or msg.data != self._queue[0].id:
            return
        self._queue.pop(0)
        self._since = time.monotonic()
        if self._queue:
            self._publish_maneuver()
        else:
            self.node.get_logger().info('Durchfahrt fertig.')
            self._finish()

    def cb_tick(self):
        if not self._queue:
            return
        if time.monotonic() - self._since > STEP_TIMEOUT_S:
            self.node.get_logger().warn(
                f'Durchfahrt: Schritt dauert laenger als {STEP_TIMEOUT_S:.0f} s -- Halt.')
            self.pub_maneuver.publish(self._straight(0.0))
            self._finish()
            return
        # Zyklisch, falls control_driving eine Nachricht verpasst hat.
        self._publish_maneuver()

    # --- Grundbewegungen ---

    def _new_maneuver(self, kind, max_vel):
        self._maneuver_id += 1
        m = Maneuver()
        m.header.frame_id = ODOM_FRAME
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.id = self._maneuver_id
        m.type = kind
        m.max_vel = float(max_vel)
        return m

    def _rotate(self, yaw, max_vel=0.0):
        m = self._new_maneuver(Maneuver.ROTATE, max_vel)
        m.yaw = float(yaw)
        return m

    def _straight(self, distance, max_vel=0.0):
        m = self._new_maneuver(Maneuver.STRAIGHT, max_vel)
        m.distance = float(distance)
        return m

    def _path_to(self, xy, reverse, max_vel=0.0):
        m = self._new_maneuver(Maneuver.PATH, max_vel)
        m.reverse = bool(reverse)
        m.path.header = m.header
        pose = PoseStamped()
        pose.header = m.header
        pose.pose.position.x, pose.pose.position.y = float(xy[0]), float(xy[1])
        pose.pose.orientation.w = 1.0
        m.path.poses = [pose]
        return m

    # --- Hilfsmethoden ---

    def _publish_maneuver(self):
        self.pub_maneuver.publish(self._queue[0])

    def _finish(self):
        self._queue = []
        self.pub_passage.publish(Bool(data=False))

    def _robot_pose(self):
        try:
            tf = self.node.tf_buffer.lookup_transform(ODOM_FRAME, ROBOT_FRAME, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.node.get_logger().warn(f'Keine Roboterpose fuer die Durchfahrt: {e}')
            return None
        return pose2d_from_transform(tf)
