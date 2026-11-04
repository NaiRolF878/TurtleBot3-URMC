"""Faehrt durch die Tuer in den Tunnel, sucht den Ausgang und meldet, wenn der Bot draussen ist.

Ein:  /control/mode, /global_costmap/costmap, /scan, /detect/lines_lost, /tunnel/path_result, TF
Aus:  /tunnel/explore_path (Path, odom), /tunnel/door (PolygonStamped, odom: Tuerkanten und
      Punkt innen), /tunnel/outside (Bool); Pfade rechnet planner_server (ComputePathToPose)
"""
import math
import time

import cv2
import numpy as np

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String, UInt8
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from Follow_Lane.src import tunnel_geometry as geo
from Follow_Lane.src.common import (
    ControlType, control_mode_from_msg, path_blocked, pose2d_from_transform, run_node)

ODOM_FRAME = 'odom'
ROBOT_FRAME = 'base_footprint'
TICK_S = 0.5
# Antwortet planner_server so lange nicht, gilt die Anfrage als verloren.
PLAN_TIMEOUT_S = 5.0
# Karten aus der Zeit vor dem Leeren (lane_costmap_node, 0,3 s) nicht verwenden; die
# global_costmap sendet mit 1 Hz.
MAP_SETTLE_S = 1.5
# So lange (s) muss der Bot durchgehend ausserhalb der Huelle sein.
EXIT_HOLD_S = 1.0
# Um diesen Radius (m) gilt eine Stelle als dieselbe: xy_goal_tolerance des Reglers.
SAME_TARGET_M = 0.15

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

    Zielwahl in fester Reihenfolge: (1) Strahlen durch die Huelle nach aussen zeigen auf eine
    Oeffnung, nicht die am Eingang. (2) Naechste Frontier in der Huelle: Graues dort ist Schatten
    hinter Hindernissen. (3) Frontier jenseits der Eingangslinie, weit weg vom Eingang. (4) Der
    weiteste erreichbare Punkt. Besuchte und gescheiterte Stellen werden nicht wieder gewaehlt.

    Ein Ziel wechselt nur bei Ankunft, wenn es unerreichbar wird oder wenn eine Oeffnung
    auftaucht. Keine Neuplanung im Takt, die Wege sollen im Tunnel nicht springen: neu geplant
    wird zum selben Ziel nur, wenn eine gesperrte Zelle auf dem Pfad liegt oder der Regler
    einmal abbricht. Anders als im Obstacle-Modus gibt es keine Rueckwaertssperre, der Bot muss
    aus Sackgassen wieder herauskommen.
    """

    def __init__(self):
        super().__init__('tunnel_explorer_node')

        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)
        self.sub_costmap = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.cb_costmap, 1)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)
        self.sub_lines_lost = self.create_subscription(
            Bool, '/detect/lines_lost', self.cb_lines_lost, 1)
        self.sub_path_result = self.create_subscription(
            String, '/tunnel/path_result', self.cb_path_result, 1)

        self.pub_explore_path = self.create_publisher(Path, '/tunnel/explore_path', 1)
        self.pub_door = self.create_publisher(PolygonStamped, '/tunnel/door', 1)
        self.pub_outside = self.create_publisher(Bool, '/tunnel/outside', 1)
        self.plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        self.timer = self.create_timer(TICK_S, self.cb_timer)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.control_mode = None
        self.latest_costmap = None
        self.latest_scan = None
        self._saw_lines = False
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
        self.control_mode = new_mode

    def cb_costmap(self, grid):
        if self._tunnel_since is not None \
                and time.monotonic() - self._tunnel_since >= MAP_SETTLE_S:
            self.latest_costmap = grid

    def cb_scan(self, scan):
        self.latest_scan = scan

    def cb_lines_lost(self, msg):
        if not msg.data:
            self._saw_lines = True

    def cb_path_result(self, msg):
        if self.control_mode != ControlType.Tunnel or self._target is None:
            return
        if msg.data == 'erreicht':
            self._finish_target(failed=False)
        elif not self._retried:
            self._retried = True
            self._request_path(self._target)
        else:
            self._finish_target(failed=True)

    def cb_timer(self):
        if self.control_mode != ControlType.Tunnel or self.latest_costmap is None:
            return
        pose = self._robot_pose()
        if pose is None:
            return
        grid, res, origin = self._grid()
        self._update_walls(grid, res, origin, pose)
        hull = geo.convex_hull(self._walls_xy())
        self._update_outside(hull, pose)

        if self._planning_since is not None:
            if time.monotonic() - self._planning_since < PLAN_TIMEOUT_S:
                return
            self.get_logger().warn('planner_server antwortet nicht -- neue Anfrage.',
                                   throttle_duration_sec=5.0)
            self._planning_since = None

        if self._phase == ENTERING:
            self._enter_step(pose)
            return
        self._search_step(grid, res, origin, pose, hull)

    def cb_plan_response(self, future, target):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Pfad-Anfrage vom planner_server abgelehnt.',
                                   throttle_duration_sec=5.0)
            self._planning_since = None
            return
        goal_handle.get_result_async().add_done_callback(
            lambda f, t=target: self.cb_plan_result(f, t))

    def cb_plan_result(self, future, target):
        self._planning_since = None
        # Der Modus oder das Ziel kann waehrend der Planung gewechselt haben.
        if self.control_mode != ControlType.Tunnel or target != self._target:
            return
        result = future.result()
        path = result.result.path if result is not None else None
        if path is None or not path.poses:
            self.get_logger().warn(f'planner_server findet keinen Pfad zu {self._kind} '
                                   f'x={target[0]:.2f} y={target[1]:.2f}.',
                                   throttle_duration_sec=2.0)
            self._finish_target(failed=True)
            return
        self._path = path
        self.pub_explore_path.publish(path)

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
        if target is None and self._done:
            self.get_logger().warn('Besuchte Stellen vergessen und neu gesucht.',
                                   throttle_duration_sec=5.0)
            self._done = []
            target, kind = self._frontier_target(grid, free, res, origin, pose, known_hull)
        if target is None:
            self.get_logger().warn('Kein erreichbares Ziel im Tunnel.', throttle_duration_sec=2.0)
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
            if target is not None and not self._is_done(target):
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
            inside = [t for t in targets if cv2.pointPolygonTest(hull, t, False) >= 0]
            if inside:
                return min(inside, key=lambda t: np.hypot(*(t - robot))), 'Frontier in der Huelle'

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
        walls = geo.connected_walls(grid, rc)
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
        self.get_logger().info(f'Ziel ({kind}): x={target[0]:.2f} y={target[1]:.2f}.')
        self._request_path(target)

    def _finish_target(self, failed):
        if self._kind == 'Tuer':
            if failed:
                self._door = None
            else:
                self._phase = EXPLORING
        else:
            self._done.append(self._target)
        if failed:
            self.get_logger().warn(f'{self._kind} gilt als unerreichbar.', throttle_duration_sec=2.0)
        self._target = None
        self._kind = None
        self._path = None

    def _is_done(self, point):
        return any(math.hypot(point[0] - d[0], point[1] - d[1]) < SAME_TARGET_M for d in self._done)

    def _path_blocked(self):
        """Liegt eine gesperrte Zelle auf dem restlichen Pfad? Nur dann wird neu geplant."""
        if self._path is None or self._planning_since is not None:
            return False
        pose = self._robot_pose()
        if pose is None:
            return False
        grid, res, origin = self._grid()
        points = np.array([[p.pose.position.x, p.pose.position.y] for p in self._path.poses])
        return path_blocked(points, pose[:2], grid, res, origin)

    def _request_path(self, target):
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
        self._planning_since = time.monotonic()
        self.plan_client.send_goal_async(goal_msg).add_done_callback(
            lambda f, t=target: self.cb_plan_response(f, t))

    # --- Hilfsmethoden ---

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
        self._path = None
        self._done = []
        self._planning_since = None
        self._scanner = None

    def _grid(self):
        msg = self.latest_costmap
        info = msg.info
        grid = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        return grid, info.resolution, np.array([info.origin.position.x, info.origin.position.y])

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

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down tunnel_explorer_node Node.')


def main(args=None):
    run_node(tunnel_explorer_node, args)


if __name__ == '__main__':
    main()
