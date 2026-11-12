"""Faehrt durch die Tuer in den Tunnel, sucht den Ausgang und meldet, wenn der Bot draussen ist.

Ein:  /control/mode, /global_costmap/costmap, /scan_corrected, /detect/lines_lost,
      /tunnel/path_result (erreicht / kollision / abgebrochen), /control/maneuver_sequence, TF
Aus:  /tunnel/explore_path (Path, odom), /tunnel/door (PolygonStamped, odom: Tuerkanten und
      Punkt innen), /tunnel/outside (Bool), /nav2/blocked (PointCloud2, odom: gesperrte
      Engstellen), /control/maneuver_request (Engstelle, Freifahren); Pfade rechnet
      planner_server (ComputePathToPose). Fahrtprotokoll in $TUNNEL_LOG_DIR/<Zeit>/.
"""
import json
import math
import os
import time

import cv2
import numpy as np

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Empty, Header, String, UInt8
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from follow_lane_msg.msg import ManeuverRequest, ManeuverStatus
from Follow_Lane.src import passage_geometry as pg
from Follow_Lane.src import tunnel_geometry as geo
from Follow_Lane.src.common import (
    ControlType, control_mode_from_msg, path_blocked, pose2d_from_transform, run_node)

ODOM_FRAME = 'odom'
ROBOT_FRAME = 'base_footprint'
TICK_S = 0.5
# Antwortet planner_server so lange nicht, gilt die Anfrage als verloren.
PLAN_TIMEOUT_S = 5.0
# Karten aus der Zeit vor dem Leeren (lane_costmap_node, 0,3 s) nicht verwenden; die
# global_costmap sendet mit 2 Hz.
MAP_SETTLE_S = 1.5
# So lange (s) muss der Bot durchgehend ausserhalb der Huelle sein.
EXIT_HOLD_S = 1.0
# Um diesen Radius (m) gilt eine Stelle als dieselbe: xy_goal_tolerance des Reglers.
SAME_TARGET_M = 0.15
# Engstellen naeher als das (m) sind dieselbe.
SAME_GAP_M = 0.05
# Liegt der Start einer Engstelle so nah (m), beginnen die Grundbewegungen sofort.
PASSAGE_NEAR_M = 0.10
# Besuchte und gescheiterte Ziele werden so lange (s) gemieden, nicht fuer immer (wie HKA).
AVOID_S = 15.0
# Neuplanung zum selben Ziel im Takt (s); der neue Pfad gilt nur bei gleicher Richtung
# (passage_geometry.same_direction) oder wenn der alte blockiert ist.
REPLAN_PERIOD_S = 1.0
# So oft je Ziel wird nach Kollisionsgefahr freigefahren, danach gilt das Ziel als gescheitert.
UNWEDGE_MAX = 2
# Tunnel-Uhr: Reglement 60 s fuer die Ausfahrt. Ab TUNNEL_HURRY_S zaehlt bei der Zielwahl nur
# noch die Naehe, nicht mehr der Gewinn.
TUNNEL_HURRY_S = 40.0
# Reglerwerte im Tunnel, beim Verlassen zurueck auf NORMAL (= nav2_params.yaml): lockerer
# Fortschritt (Mindestweg m in Sekunden) und langsamer zwischen den engen Waenden.
TUNNEL_CONTROLLER = {
    'progress_checker.required_movement_radius': 0.05,
    'progress_checker.movement_time_allowance': 6.0,
    'FollowPath.desired_linear_vel': 0.10,
}
NORMAL_CONTROLLER = {
    'progress_checker.required_movement_radius': 0.1,
    'progress_checker.movement_time_allowance': 3.0,
    'FollowPath.desired_linear_vel': 0.12,
}
# Fahrtprotokoll: Verzeichnis aus dieser Umgebungsvariable (launchers/nav2_obstacle.sh).
LOG_DIR_ENV = 'TUNNEL_LOG_DIR'

ENTERING = 'Einfahrt'
EXPLORING = 'Suche'
OPENING = 'Oeffnung'


class tunnel_explorer_node(Node):
    """Einfahrt: die naechste Tuer voraus im Scan, Ziel so weit dahinter, dass der Bot drin ist.
    Stand beim Moduswechsel gar keine Linie im Bild, ist er schon im Eingang oder im Tunnel und
    die Einfahrt entfaellt.

    Die Wand-Huelle waechst aus den Tuerkanten (sonst den naechsten Waenden links und rechts)
    ueber alle damit verbundenen Wandzellen. Raumwaende und freistehende Hindernisse gehoeren
    nicht dazu. Drinnen liegt der Bot in der Huelle, draussen liegen alle Waende auf einer Seite:
    das kommt ohne Masse der Enclosure aus. /tunnel/outside wird erst gesendet, nachdem der Bot
    einmal drin war, ein Start vor dem Tunnel springt also nicht zurueck.

    Die Tuer wird nur einmal je Tunnelfahrt gesucht und bleibt danach fest, auch wenn das
    Ziel dahinter scheitert.

    Im Tunnel faehrt der Regler langsamer und prueft den Fortschritt lockerer
    (TUNNEL_CONTROLLER); ausserhalb gilt wieder nav2_params.yaml (NORMAL_CONTROLLER).

    Zielwahl in fester Reihenfolge: (1) Strahlen durch die Huelle nach aussen zeigen auf eine
    Oeffnung, nicht die am Eingang. (2) Frontier in der Huelle mit Unbekanntem an der Wand
    (tunnel_geometry.WALL_BAND_M): Gewinn gegen Umweg, ab TUNNEL_HURRY_S die naechste. Graues
    tiefer drin ist Schatten hinter Hindernissen und zaehlt nicht. (3) Frontier jenseits der
    Eingangslinie, weit weg vom Eingang. (4) Der weiteste erreichbare Punkt. Besuchte und
    gescheiterte Ziele werden AVOID_S lang gemieden.

    Jeder neue Pfad wird vor dem Losfahren mit dem Umriss geprueft (passage_geometry): passt der
    Bot an einer Engstelle nur ausgerichtet, faehrt RPP bis zum Start der Geraden, dann
    Grundbewegungen (switch_control) hindurch, danach wird zum selben Ziel neu geplant. Passt er
    gar nicht oder ist die Durchfahrt dort gescheitert, wird die Engstelle gesperrt
    (/nav2/blocked, entrance_layer beider Karten) bis zum Ende der Tunnelfahrt oder Taste L.

    Abbruch des Reglers: bei Kollisionsgefahr freifahren (hoechstens UNWEDGE_MAX je Ziel), sonst
    einmal neu planen; danach gilt das Ziel als gescheitert. Ein Ziel wechselt bei Ankunft, wenn
    es unerreichbar wird oder wenn eine Oeffnung auftaucht. Zum selben Ziel wird im Takt
    REPLAN_PERIOD_S neu geplant, der neue Pfad gilt aber nur bei gleicher Richtung.
    """

    def __init__(self):
        super().__init__('tunnel_explorer_node')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)
        self.sub_costmap = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.cb_costmap, 1)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan_corrected', self.cb_scan, qos_profile_sensor_data)
        self.sub_lines_lost = self.create_subscription(
            Bool, '/detect/lines_lost', self.cb_lines_lost, 1)
        self.sub_path_result = self.create_subscription(
            String, '/tunnel/path_result', self.cb_path_result, 1)
        self.sub_clear_maps = self.create_subscription(
            Empty, '/control/clear_maps', self.cb_clear_maps, 1)
        self.sub_sequence = self.create_subscription(
            ManeuverStatus, '/control/maneuver_sequence', self.cb_sequence, latched)

        self.pub_explore_path = self.create_publisher(Path, '/tunnel/explore_path', 1)
        self.pub_door = self.create_publisher(PolygonStamped, '/tunnel/door', 1)
        self.pub_outside = self.create_publisher(Bool, '/tunnel/outside', 1)
        self.pub_blocked = self.create_publisher(PointCloud2, '/nav2/blocked', 1)
        self.pub_request = self.create_publisher(ManeuverRequest, '/control/maneuver_request', 1)
        self.plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.controller_params = self.create_client(SetParameters, '/controller_server/set_parameters')

        self.timer = self.create_timer(TICK_S, self.cb_timer)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.control_mode = None
        self.latest_costmap = None
        self.latest_scan = None
        self._saw_lines = False
        self._log = None
        self._reset()

        self.get_logger().info('Started tunnel_explorer_node Node.')

    # --- Callbacks ---

    def cb_control_mode(self, msg):
        new_mode = control_mode_from_msg(self, msg)
        if new_mode is None or new_mode == self.control_mode:
            return
        if new_mode == ControlType.TunnelEntry:
            self._saw_lines = False
        if new_mode == ControlType.Tunnel:
            self._reset()
            self.latest_costmap = None
            self._tunnel_since = time.monotonic()
            self._phase = ENTERING if self._saw_lines else EXPLORING
            self._set_controller_params(TUNNEL_CONTROLLER, 'Tunnel')
            self._open_log()
        elif self.control_mode == ControlType.Tunnel:
            self._set_controller_params(NORMAL_CONTROLLER, 'normal')
            self._close_log(f'Modus {new_mode.name}')
            # Leere Wolke: die entrance_layer tragen sonst die Sperren weiter ein.
            self._blocked = []
            self._publish_blocked()
        self.control_mode = new_mode

    def cb_clear_maps(self, _msg):
        """Taste L im Tunnel: Neustart vor dem Tunnel, Modus bleibt Tunnel. Alles Gemerkte
        (Tuer, Waende, Ziele) weg, die Karte wird erst nach MAP_SETTLE_S wieder gelesen (bis
        dahin hat lane_costmap_node die Costmaps geleert). Einfahrt wie nach TunnelEntry: Tuer
        im Scan suchen. Die Tunnel-Reglerwerte bleiben."""
        if self.control_mode != ControlType.Tunnel:
            return
        self._reset()
        self.latest_costmap = None
        self._tunnel_since = time.monotonic()
        self._phase = ENTERING
        self._event('taste_l')
        self.get_logger().info('Taste L: Tunnelmodus zurueckgesetzt -- Einfahrt neu.')

    def cb_costmap(self, grid):
        if self._tunnel_since is not None \
                and time.monotonic() - self._tunnel_since >= MAP_SETTLE_S:
            self.latest_costmap = grid

    def cb_scan(self, scan):
        self.latest_scan = scan
        if self._log is not None:
            self._log.write('scans', {
                't': time.monotonic(),
                'stamp': scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9,
                'angle_min': scan.angle_min, 'angle_increment': scan.angle_increment,
                'range_min': scan.range_min,
                # mm; -1: kein Echo (inf), 0: ungueltig
                'mm': [int(r * 1000) if math.isfinite(r) else (-1 if r > 0 else 0)
                       for r in scan.ranges]})

    def cb_lines_lost(self, msg):
        if not msg.data:
            self._saw_lines = True

    def cb_path_result(self, msg):
        if self.control_mode != ControlType.Tunnel or self._target is None \
                or self._waiting_for is not None:
            return
        self._event('path_result', result=msg.data)
        if msg.data == 'erreicht':
            if self._pending_passage is not None:
                self._request(ManeuverRequest.PASSAGE)
            else:
                self._finish_target(failed=False)
            return
        self._pending_passage = None
        if msg.data == 'kollision' and self._unwedges < UNWEDGE_MAX:
            self._unwedges += 1
            self._request(ManeuverRequest.UNWEDGE)
        elif not self._retried:
            self._retried = True
            self._request_path(self._target)
        else:
            self._finish_target(failed=True)

    def cb_sequence(self, msg):
        if msg.active or self._waiting_for is None:
            return
        kind, self._waiting_for = self._waiting_for, None
        self._event('maneuver_done', kind=int(kind), success=msg.success, reason=msg.reason)
        if self.control_mode != ControlType.Tunnel or self._target is None:
            return
        if kind == ManeuverRequest.PASSAGE:
            passage, self._pending_passage = self._pending_passage, None
            if msg.success:
                self.get_logger().info(f'Engstelle durchfahren -- plane neu zu {self._kind}.')
            elif passage is not None:
                self._failed_gaps.append(passage[2])
                self._block(passage[2], passage[3], passage[4],
                            f'Durchfahrt gescheitert ({msg.reason})')
        elif not msg.success:
            self.get_logger().warn(f'Freifahren nicht moeglich: {msg.reason}.')
        self._retried = False
        self._path = None
        self._request_path(self._target)

    def cb_timer(self):
        if self.control_mode != ControlType.Tunnel or self.latest_costmap is None:
            return
        self._publish_blocked()
        pose = self._robot_pose()
        if pose is None:
            return
        if self._log is not None:
            self._log.write('poses', {'t': time.monotonic(), 'pose': list(pose)})
        grid, res, origin = self._grid()
        self._update_walls(grid, res, origin, pose)
        hull = geo.convex_hull(self._walls_xy())
        self._update_outside(hull, pose)
        if self._waiting_for is not None:
            return

        if self._planning_since is not None:
            if time.monotonic() - self._planning_since < PLAN_TIMEOUT_S:
                return
            self.get_logger().warn('planner_server antwortet nicht -- neue Anfrage.',
                                   throttle_duration_sec=5.0)
            self._planning_since = None

        if self._phase == ENTERING:
            self._enter_step(pose)
        else:
            self._search_step(grid, res, origin, pose, hull)
        self._replan_periodic()

    def cb_plan_response(self, future, target, periodic):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Pfad-Anfrage vom planner_server abgelehnt.',
                                   throttle_duration_sec=5.0)
            self._planning_since = None
            return
        goal_handle.get_result_async().add_done_callback(
            lambda f, t=target, p=periodic: self.cb_plan_result(f, t, p))

    def cb_plan_result(self, future, target, periodic):
        self._planning_since = None
        # Der Modus oder das Ziel kann waehrend der Planung gewechselt haben.
        if self.control_mode != ControlType.Tunnel or target != self._target \
                or self._waiting_for is not None:
            return
        result = future.result()
        path = result.result.path if result is not None else None
        if path is None or not path.poses:
            if periodic and self._path is not None:
                return
            self.get_logger().warn(f'planner_server findet keinen Pfad zu {self._kind} '
                                   f'x={target[0]:.2f} y={target[1]:.2f}.',
                                   throttle_duration_sec=2.0)
            self._finish_target(failed=True)
            return
        pose = self._robot_pose()
        if pose is None or self.latest_costmap is None:
            return
        points = np.array([[p.pose.position.x, p.pose.position.y] for p in path.poses])
        if periodic and self._path is not None and not self._path_blocked() \
                and not pg.same_direction(self._path_xy(), points, pose[:2]):
            return
        self._drive(path, points, pose)

    # --- Einfahrt ---

    def _enter_step(self, pose):
        if self._target is not None:
            if self._path_blocked():
                self._request_path(self._target)
            return
        door = self._find_door(pose)
        if door is None:
            self.get_logger().warn('Keine Tunneloeffnung im Scan gefunden -- Bot wartet.',
                                   throttle_duration_sec=2.0)
            return
        a, b, center, width, normal = door
        self._door = (a, b, center, normal, width)
        self._seeds = np.vstack([self._seeds, a, b])
        target = center + normal * geo.DOOR_DEPTH_M
        self._publish_door(a, b, target)
        self.get_logger().info(f'Tuer gefunden, {width:.2f} m breit: x={center[0]:.2f} '
                               f'y={center[1]:.2f}.')
        self._set_target((float(target[0]), float(target[1])), 'Tuer')

    def _find_door(self, pose):
        """Naechste Oeffnung vor dem Bot im odom-Frame."""
        scan = self.latest_scan
        scanner = self._scanner_pose(scan)
        if scan is None or scanner is None:
            return None
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
        angles = np.arctan2(np.sin(angles), np.cos(angles))
        ranges = np.where(ranges > scan.range_min, ranges, np.nan)
        in_robot = [o for o in geo.find_openings(angles, ranges)
                    if (geo.transform(scanner, o[2][None])[0][0]) > 0.0]
        if not in_robot:
            return None
        a, b, center, width, normal = in_robot[0]
        world = (lambda p: geo.transform(pose, geo.transform(scanner, p[None]))[0])
        out = world(center + normal) - world(center)
        return world(a), world(b), world(center), width, out / np.hypot(*out)

    def _publish_door(self, a, b, inside):
        msg = PolygonStamped()
        msg.header.frame_id = ODOM_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.polygon.points = [Point32(x=float(p[0]), y=float(p[1]), z=0.0) for p in (a, b, inside)]
        self.pub_door.publish(msg)

    # --- Suche ---

    def _search_step(self, grid, res, origin, pose, hull):
        free = geo.reachable_free(grid, res, origin, pose[:2])
        inside = hull is not None and geo.bot_inside(hull, pose)
        opening = None
        if free is not None and inside:
            opening = self._opening_target(hull, free, res, origin, pose)

        if self._target is not None:
            if opening is not None and self._kind != OPENING:
                self._set_target(opening, OPENING)
            elif self._path_blocked():
                self._request_path(self._target)
            return

        if free is None:
            self.get_logger().warn('Bot steht in keiner befahrbaren Flaeche -- kein Ziel.',
                                   throttle_duration_sec=2.0)
            return
        if opening is not None:
            self._set_target(opening, OPENING)
            return
        known_hull = hull if inside or self._door is not None else None
        target, kind = self._frontier_target(grid, free, res, origin, pose, known_hull)
        if target is None:
            self.get_logger().warn('Kein Weg nach draussen gefunden -- Bot wartet '
                                   '(Taste L: Tunnel neu).', throttle_duration_sec=10.0)
            return
        self._set_target(target, kind)

    def _opening_target(self, hull, free, res, origin, pose):
        """Zielzelle hinter der breitesten Oeffnung der Huelle, nicht am Eingang."""
        scan = self.latest_scan
        scanner = self._scanner_pose(scan)
        if scan is None or scanner is None:
            return None
        angles, ranges = self._world_rays(scan, scanner, pose)
        sensor = geo.transform(pose, np.array([scanner[:2]]))[0]
        for center, width, direction in geo.hull_openings(hull, sensor, angles, ranges):
            if self._door is not None and np.hypot(*(center - self._door[2])) < self._door[4]:
                continue
            entrance, inward = self._entrance(pose)
            if (center - entrance) @ inward <= 0.0:
                continue
            target = geo.nearest_free(free, res, origin, center + direction * geo.BOT_LENGTH, width)
            # Liegt das Ziel noch in der Huelle, war die Luecke fuer NavFn zu eng.
            if target is not None and cv2.pointPolygonTest(hull, target, False) < 0 \
                    and not self._is_done(target):
                return target
        return None

    def _world_rays(self, scan, scanner, pose):
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment + scanner[2] + pose[2]
        angles = np.arctan2(np.sin(angles), np.cos(angles))
        return angles, np.where(ranges > scan.range_min, ranges, np.nan)

    def _frontier_target(self, grid, free, res, origin, pose, hull):
        targets = geo.frontier_targets(geo.frontier_mask(free, grid), res, origin)
        targets = [t for t in targets if not self._is_done(t)
                   and math.hypot(t[0] - pose[0], t[1] - pose[1]) > SAME_TARGET_M]
        robot = np.array(pose[:2])

        if hull is not None:
            band = geo.wall_band_unknown(grid, hull, res, origin)
            scored = [(geo.frontier_gain(band, res, origin, t), t) for t in targets
                      if cv2.pointPolygonTest(hull, t, False) >= 0]
            scored = [(gain, t) for gain, t in scored if gain > 0.0]
            if scored:
                if self._hurry():
                    best = min(scored, key=lambda gt: np.hypot(*(gt[1] - robot)))
                else:
                    best = max(scored, key=lambda gt: geo.GAIN_WEIGHT * gt[0]
                               - np.hypot(*(gt[1] - robot)))
                return best[1], 'Frontier an der Wand'

        entrance, inward = self._entrance(pose)
        ahead = [t for t in targets if (np.array(t) - entrance) @ inward > 0.0]
        if ahead:
            # Weit weg vom Eingang zaehlt mehr als der kurze Weg.
            best = max(ahead, key=lambda t: np.hypot(*(t - entrance)) - 0.3 * np.hypot(*(t - robot)))
            return best, 'Frontier'

        rows, cols = np.nonzero(free)
        xs = origin[0] + (cols + 0.5) * res
        ys = origin[1] + (rows + 0.5) * res
        keep = ((np.column_stack([xs, ys]) - entrance) @ inward) > 0.0
        if not keep.any():
            return None, None
        xs, ys = xs[keep], ys[keep]
        i = np.argmax(np.hypot(xs - entrance[0], ys - entrance[1]))
        far = (float(xs[i]), float(ys[i]))
        if math.hypot(far[0] - pose[0], far[1] - pose[1]) <= SAME_TARGET_M or self._is_done(far):
            return None, None
        return far, 'weitester Punkt'

    def _entrance(self, pose):
        """Eingangslinie als (Punkt, Normale nach innen)."""
        if self._door is not None:
            return self._door[2], self._door[3]
        if self._start_pose is None:
            self._start_pose = pose
        yaw = self._start_pose[2]
        return np.array(self._start_pose[:2]), np.array([math.cos(yaw), math.sin(yaw)])

    # --- Huelle ---

    def _update_walls(self, grid, res, origin, pose):
        seeds = self._seeds
        if len(seeds) == 0 and self._phase == EXPLORING and self._door is None:
            rc = geo.side_wall_seeds(grid, res, origin, pose)
            seeds = origin + (rc[:, ::-1] + 0.5) * res if len(rc) else seeds
        if len(seeds) == 0:
            return
        rc = np.floor((np.asarray(seeds) - origin) / res).astype(np.int64)[:, ::-1]
        walls = geo.connected_walls(grid, rc, res)
        rows, cols = np.nonzero(walls)
        if rows.size == 0:
            return
        # In odom gemerkt: die mitfahrende Karte verliert, was hinten herausrollt.
        points = origin + (np.column_stack([cols, rows]) + 0.5) * res
        self._walls.update(map(tuple, np.round(points, 3).tolist()))
        self._seeds = np.array(list(self._walls))

    def _walls_xy(self):
        return np.array(list(self._walls), dtype=np.float32) if self._walls else np.empty((0, 2))

    def _update_outside(self, hull, pose):
        if hull is None:
            return
        if geo.bot_inside(hull, pose):
            self._was_inside = True
            self._outside_since = None
            return
        if not self._was_inside or not geo.bot_outside(hull, pose):
            self._outside_since = None
            return
        now = time.monotonic()
        if self._outside_since is None:
            self._outside_since = now
        elif now - self._outside_since >= EXIT_HOLD_S:
            self.pub_outside.publish(Bool(data=True))

    # --- Ziele ---

    def _set_target(self, target, kind):
        self._target = target
        self._kind = kind
        self._retried = False
        self._unwedges = 0
        self._pending_passage = None
        self._path = None
        self.get_logger().info(f'Ziel ({kind}): x={target[0]:.2f} y={target[1]:.2f}.')
        self._event('target', kind=kind, xy=list(target))
        self._request_path(target)

    def _finish_target(self, failed):
        if self._kind == 'Tuer':
            # Die Tuer bleibt auch nach einem Fehlschlag fest (neu gesucht faende der Scan
            # Luecken zwischen beliebigen Objekten). Weiter mit der Suche.
            self._phase = EXPLORING
        else:
            self._done.append((self._target, time.monotonic() + AVOID_S))
        if failed:
            self.get_logger().warn(f'{self._kind} gilt als unerreichbar.', throttle_duration_sec=2.0)
        self._event('target_done', kind=self._kind, failed=failed)
        self._target = None
        self._kind = None
        self._path = None
        self._pending_passage = None

    def _drive(self, path, points, pose):
        """Pfad pruefen (Engstellen), dann abfahren, bis zum Start einer Engstelle fahren oder
        die Engstelle sperren; neu geplant wird dann im Takt, sobald die Sperre in der Karte
        steht."""
        grid, res, origin = self._grid()
        verdict, info = pg.check_path(points, grid, res, origin, pose[:2])
        if verdict == pg.ALIGN and self._near(info[2], self._failed_gaps):
            verdict, info = pg.BLOCK, info[2:]
        self._event('path', verdict=verdict, points=len(points))
        if verdict == pg.BLOCK:
            self._block(*info, 'Bot passt nicht hindurch')
            self._path = None
            return
        if verdict == pg.ALIGN:
            start = info[0]
            self._pending_passage = info
            if math.hypot(start[0] - pose[0], start[1] - pose[1]) < PASSAGE_NEAR_M:
                self._path = None
                self._request(ManeuverRequest.PASSAGE)
                return
            path = self._path_msg(pg.truncate_at(points, start))
            self.get_logger().info(f'Engstelle {info[4]:.2f} m: erst ausrichten, Start '
                                   f'x={start[0]:.2f} y={start[1]:.2f}.')
        else:
            self._pending_passage = None
        self._path = path
        self.pub_explore_path.publish(path)

    def _request(self, kind):
        """Grundbewegungen bei switch_control anfordern; weiter geht es in cb_sequence."""
        msg = ManeuverRequest(type=kind)
        if kind == ManeuverRequest.PASSAGE:
            msg.path = self._path_msg(self._pending_passage[:2])
        self._waiting_for = kind
        self._event('maneuver', kind=int(kind))
        self.pub_request.publish(msg)

    def _block(self, center, axis, width, why):
        if self._near(center, [line.mean(axis=0) for line in self._blocked]):
            return
        self._blocked.append(pg.block_points(center, axis, width))
        self._event('block', center=[float(c) for c in center], width=float(width), why=why)
        self.get_logger().warn(f'Engstelle {width:.2f} m bei x={center[0]:.2f} y={center[1]:.2f}: '
                               f'{why} -- gesperrt, suche andere Route.')

    def _publish_blocked(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = ODOM_FRAME
        points = [(float(x), float(y), 0.0) for line in self._blocked for x, y in line]
        self.pub_blocked.publish(point_cloud2.create_cloud_xyz32(header, points))

    def _is_done(self, point):
        now = time.monotonic()
        return any(until > now and math.hypot(point[0] - d[0], point[1] - d[1]) < SAME_TARGET_M
                   for d, until in self._done)

    def _hurry(self):
        return self._tunnel_since is not None \
            and time.monotonic() - self._tunnel_since >= TUNNEL_HURRY_S

    def _path_blocked(self):
        """Liegt eine gesperrte Zelle auf dem restlichen Pfad? Dann wird neu geplant."""
        if self._path is None or self._planning_since is not None:
            return False
        pose = self._robot_pose()
        if pose is None:
            return False
        grid, res, origin = self._grid()
        return path_blocked(self._path_xy(), pose[:2], grid, res, origin)

    def _replan_periodic(self):
        if self._target is None or self._planning_since is not None \
                or self._waiting_for is not None or self._pending_passage is not None \
                or time.monotonic() - self._last_plan < REPLAN_PERIOD_S:
            return
        self._request_path(self._target, periodic=True)

    def _request_path(self, target, periodic=False):
        if not self.plan_client.server_is_ready():
            self.get_logger().warn(
                'compute_path_to_pose Action-Server nicht erreichbar -- laeuft planner_server?',
                throttle_duration_sec=5.0)
            return
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal.header.frame_id = ODOM_FRAME
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.pose.position.x = target[0]
        goal_msg.goal.pose.position.y = target[1]
        goal_msg.goal.pose.orientation.w = 1.0
        goal_msg.use_start = False
        self._planning_since = self._last_plan = time.monotonic()
        self.plan_client.send_goal_async(goal_msg).add_done_callback(
            lambda f, t=target, p=periodic: self.cb_plan_response(f, t, p))

    # --- Hilfsmethoden ---

    def _set_controller_params(self, values, label):
        """Reglerwerte des controller_server zur Laufzeit umstellen (SimpleProgressChecker
        und RPP nehmen sie dynamisch an; Humble kennt nur einen progress_checker)."""
        if not self.controller_params.service_is_ready():
            self.get_logger().warn('controller_server/set_parameters nicht erreichbar -- '
                                   f'Reglerwerte ({label}) bleiben unveraendert.')
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(name, Parameter.Type.DOUBLE, float(v)).to_parameter_msg()
                          for name, v in values.items()]
        text = ', '.join(f'{name.split(".")[-1]}={v}' for name, v in values.items())

        def done(future):
            res = future.result()
            if res is None or not all(r.successful for r in res.results):
                reasons = '' if res is None else '; '.join(r.reason for r in res.results)
                self.get_logger().warn(f'Reglerwerte ({label}) nicht gesetzt: {reasons}')
            else:
                self.get_logger().info(f'Reglerwerte {label}: {text}.')

        self.controller_params.call_async(req).add_done_callback(done)

    def _reset(self):
        self._tunnel_since = None
        self._phase = EXPLORING
        self._door = None               # (a, b, mitte, normale nach innen, breite) in odom
        self._seeds = np.empty((0, 2))
        self._walls = set()             # (x, y) der Wandzellen in odom
        self._was_inside = False
        self._outside_since = None
        self._start_pose = None
        self._target = None
        self._kind = None
        self._retried = False
        self._unwedges = 0
        self._path = None
        self._done = []                 # (Ziel, gemieden bis monotonic)
        # Leere Wolke: entrance_layer traegt sonst alte Sperren aus dem Puffer wieder ein.
        self._blocked = []              # Sperrlinien (N, 2) in odom
        self._publish_blocked()
        self._failed_gaps = []          # Mitten gescheiterter Durchfahrten (odom)
        self._pending_passage = None    # Engstelle, zu deren Start RPP gerade faehrt
        self._waiting_for = None        # laufende Grundbewegungen (ManeuverRequest-Typ)
        self._planning_since = None
        self._last_plan = 0.0
        self._scanner = None

    def _grid(self):
        msg = self.latest_costmap
        info = msg.info
        grid = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        return grid, info.resolution, np.array([info.origin.position.x, info.origin.position.y])

    def _path_xy(self):
        return np.array([[p.pose.position.x, p.pose.position.y] for p in self._path.poses])

    def _path_msg(self, points):
        msg = Path()
        msg.header.frame_id = ODOM_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    @staticmethod
    def _near(point, others):
        return any(math.hypot(point[0] - o[0], point[1] - o[1]) < SAME_GAP_M for o in others)

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(ODOM_FRAME, ROBOT_FRAME, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF-Lookup fuer Roboterpose fehlgeschlagen: {e}', throttle_duration_sec=2.0)
            return None
        return pose2d_from_transform(tf)

    def _scanner_pose(self, scan):
        """Scanner im Bot-Frame (fest montiert, daher einmal nachschlagen)."""
        if self._scanner is None and scan is not None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    ROBOT_FRAME, scan.header.frame_id, rclpy.time.Time())
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f'Keine TF zum Scanner: {e}', throttle_duration_sec=2.0)
                return None
            self._scanner = pose2d_from_transform(tf)
        return self._scanner

    # --- Fahrtprotokoll ---

    def _open_log(self):
        self._close_log('neue Tunnelfahrt')
        root = os.environ.get(LOG_DIR_ENV)
        if not root:
            self.get_logger().warn(f'{LOG_DIR_ENV} nicht gesetzt -- kein Fahrtprotokoll.')
            return
        self._log = MissionLog(os.path.join(root, time.strftime('%Y%m%d-%H%M%S')),
                               self.get_logger())

    def _close_log(self, why):
        if self._log is None:
            return
        self._event('end', why=why)
        self._log.close(self._picture())
        self._log = None

    def _event(self, kind, **data):
        if self._log is not None:
            self._log.write('events', {'t': time.monotonic(), 'kind': kind, **data})

    def _picture(self):
        """Kartenbild fuers Protokoll: Costmap, Wand-Huelle, Sperren, Tuer; None ohne Karte."""
        if self.latest_costmap is None:
            return None
        grid, res, origin = self._grid()
        img = np.full(grid.shape + (3,), 255, np.uint8)
        img[grid == -1] = (160, 160, 160)
        img[(grid > 0) & (grid < 99)] = (225, 235, 255)
        img[grid == 99] = (120, 120, 200)
        img[grid == 100] = (0, 0, 0)

        def px(xy):
            return tuple(int(v) for v in np.floor((np.asarray(xy) - origin) / res))

        hull = geo.convex_hull(self._walls_xy())
        if hull is not None:
            cv2.polylines(img, [np.array([px(p) for p in hull.reshape(-1, 2)], np.int32)],
                          True, (255, 0, 0), 1)
        for line in self._blocked:
            cv2.line(img, px(line[0]), px(line[-1]), (0, 0, 255), 2)
        if self._door is not None:
            cv2.line(img, px(self._door[0]), px(self._door[1]), (0, 160, 0), 2)
        return cv2.flip(img, 0)          # Bild: y nach oben

    # --- Beenden ---

    def shut_down(self):
        self._close_log('Node beendet')
        self.get_logger().info('Shutting down tunnel_explorer_node Node.')


class MissionLog:
    """Fahrtprotokoll einer Tunnelfahrt: events/poses/scans.jsonl, map.png am Ende."""

    def __init__(self, path, logger):
        self.dir = None
        self.files = {}
        try:
            os.makedirs(path, exist_ok=True)
            self.dir = path
            logger.info(f'Fahrtprotokoll: {path}')
        except OSError as e:
            logger.warn(f'Kein Fahrtprotokoll ({e}).')

    def write(self, kind, record):
        if self.dir is None:
            return
        try:
            f = self.files.get(kind)
            if f is None:
                f = self.files[kind] = open(os.path.join(self.dir, f'{kind}.jsonl'), 'a',
                                            buffering=1)
            f.write(json.dumps(record, default=float) + '\n')
        except OSError:
            pass

    def close(self, picture):
        if self.dir is None:
            return
        if picture is not None:
            cv2.imwrite(os.path.join(self.dir, 'map.png'), picture)
        for f in self.files.values():
            f.close()
        self.files = {}


def main(args=None):
    run_node(tunnel_explorer_node, args)


if __name__ == '__main__':
    main()
