"""Ablauf aus Grundbewegungen als Teil von switch_control (Obstacle und Tunnel); der Modus bleibt.

Ein:  /control/maneuver_request (ManeuverRequest, von tunnel_explorer_node bzw.
      nav2_goal_publisher), /control/maneuver_status (ManeuverStatus, von control_driving), TF
Aus:  /control/maneuver (Maneuver), /control/maneuver_sequence (ManeuverStatus, latched:
      laeuft / erreicht / abgebrochen mit Grund)
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from follow_lane_msg.msg import Maneuver, ManeuverRequest, ManeuverStatus
from Follow_Lane.src.common import ControlType, pose2d_from_transform

ODOM_FRAME = 'odom'
ROBOT_FRAME = 'base_footprint'
TICK_PERIOD_S = 0.1
PASSAGE_VEL = 0.05
# Rueckfall, falls control_driving nichts meldet; Abbrueche meldet es selbst.
STEP_TIMEOUT_S = 20.0
SEQUENCE_MODES = (ControlType.Obstacle, ControlType.Tunnel)


class ManeuverSequence:
    """PASSAGE: zum Start der Geraden (control_driving dreht dafuer auf der Stelle und faehrt
    vor oder zurueck), auf der Stelle in Richtung Ende drehen, langsam geradeaus mit gehaltenem
    Kurs bis zum Ende. UNWEDGE: eine Freifahr-Bewegung, die control_driving selbst waehlt.

    Meldet control_driving einen Abbruch (Waechter, Sicherung, Pose veraltet, kein Ausweg), endet der
    Ablauf mit diesem Grund. Dauert ein Schritt laenger als STEP_TIMEOUT_S, haelt eine leere
    Grundbewegung den Bot an. Verlaesst der Bot die SEQUENCE_MODES, verwirft control_driving die
    Bewegung selbst."""

    def __init__(self, node):
        self.node = node

        # --- Subscriber ---
        node.create_subscription(ManeuverRequest, '/control/maneuver_request', self.cb_request, 1)
        node.create_subscription(ManeuverStatus, '/control/maneuver_status', self.cb_status, 1)

        # --- Publisher ---
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_sequence = node.create_publisher(
            ManeuverStatus, '/control/maneuver_sequence', latched)
        self.pub_maneuver = node.create_publisher(Maneuver, '/control/maneuver', 1)

        # --- Timer ---
        node.create_timer(TICK_PERIOD_S, self.cb_tick)

        # --- Zustand ---
        self._queue = []                # noch abzufahrende Grundbewegungen
        self._since = 0.0
        # Andere Nummern als ein frisch gestarteter Ablauf: control_driving startet eine id,
        # die er zuletzt beendet hat, nie erneut.
        self._maneuver_id = int(time.time()) % 1_000_000 * 1000
        self._sequence_id = 0
        self._publish(False, False, '')

    @property
    def active(self):
        return bool(self._queue)

    def cancel(self):
        if self.active:
            self.node.get_logger().info('Grundbewegungen abgebrochen: Modus verlassen.')
            self._finish(False, 'Moduswechsel')

    # --- Callbacks ---

    def cb_request(self, msg):
        log = self.node.get_logger()
        if self.active:
            # Keine Meldung: "beendet" gilt sonst fuer den laufenden Ablauf.
            log.warn('Grundbewegungen laufen schon -- neue Anfrage ignoriert.')
            return
        pose = self._robot_pose() if self.node.control_mode in SEQUENCE_MODES else None
        if pose is None:
            log.warn('Grundbewegungen abgelehnt: falscher Modus oder keine Roboterpose.')
            self._publish(False, False, 'abgelehnt: Modus oder Pose')
            return
        if msg.type == ManeuverRequest.UNWEDGE:
            self._queue = [self._new_maneuver(Maneuver.UNWEDGE, 0.0)]
            log.info('Freifahren angefordert.')
        elif msg.type == ManeuverRequest.PASSAGE and len(msg.path.poses) == 2:
            start, end = ((p.pose.position.x, p.pose.position.y) for p in msg.path.poses)
            yaw = math.atan2(end[1] - start[1], end[0] - start[0])
            length = math.dist(start, end)
            ahead = (start[0] - pose[0]) * math.cos(pose[2]) \
                + (start[1] - pose[1]) * math.sin(pose[2])
            self._queue = [self._path_to(start, reverse=ahead < 0.0), self._rotate(yaw),
                           self._straight(length, PASSAGE_VEL)]
            log.info(f'Engstelle: {"zurueck" if ahead < 0.0 else "vor"} zum Start '
                     f'x={start[0]:.2f} y={start[1]:.2f}, drehen auf {math.degrees(yaw):.0f} Grad, '
                     f'{length:.2f} m geradeaus.')
        else:
            log.warn(f'Unbekannte Anfrage (Typ {msg.type}, {len(msg.path.poses)} Punkte).')
            self._publish(False, False, 'abgelehnt: unbekannt')
            return
        self._sequence_id += 1
        self._since = time.monotonic()
        self._publish(True, False, '')
        self._publish_maneuver()

    def cb_status(self, msg):
        if not self._queue or msg.id != self._queue[0].id:
            return
        if not msg.success:
            self.node.get_logger().warn(f'Grundbewegung abgebrochen: {msg.reason}.')
            self._finish(False, msg.reason)
            return
        self._queue.pop(0)
        self._since = time.monotonic()
        if self._queue:
            self._publish_maneuver()
        else:
            self.node.get_logger().info('Grundbewegungen fertig.')
            self._finish(True, '')

    def cb_tick(self):
        if not self._queue:
            return
        if time.monotonic() - self._since > STEP_TIMEOUT_S:
            self.node.get_logger().warn(
                f'Grundbewegung dauert laenger als {STEP_TIMEOUT_S:.0f} s -- Halt.')
            self.pub_maneuver.publish(self._straight(0.0))
            self._finish(False, 'Zeit abgelaufen')
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

    def _publish(self, active, success, reason):
        self.pub_sequence.publish(ManeuverStatus(
            id=self._sequence_id, active=active, success=success, reason=reason))

    def _finish(self, success, reason):
        self._queue = []
        self._publish(False, success, reason)

    def _robot_pose(self):
        try:
            tf = self.node.tf_buffer.lookup_transform(ODOM_FRAME, ROBOT_FRAME, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.node.get_logger().warn(f'Keine Roboterpose fuer die Grundbewegungen: {e}')
            return None
        return pose2d_from_transform(tf)
