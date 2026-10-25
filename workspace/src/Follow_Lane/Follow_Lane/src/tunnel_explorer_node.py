"""Sucht im Tunnel den Ausgang und liefert den Pfad dorthin.

Ein:  /control/mode, /global_costmap/costmap (OccupancyGrid), /tunnel/entrance_pose
Aus:  /tunnel/explore_path (Path, odom); Pfade rechnet planner_server (ComputePathToPose)
"""
import math

import cv2
import numpy as np

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import UInt8
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from Follow_Lane.src.common import (
    ControlType, control_mode_from_msg, frange, irange, read_params, run_node,
    yaw_from_quaternion)


class tunnel_explorer_node(Node):
    """Frontier-Zellen der global_costmap (frei, direkt neben Unbekanntem), zu
    Clustern zusammengefasst und bewertet nach "weit weg vom Eingang" und "schnell
    erreichbar". Zum besten Punkt laesst planner_server einen Pfad rechnen.

    Nur Frontier jenseits der Eingangslinie (in Einfahrtrichtung) kommen in Frage.
    switch_control schaltet schon vor der Oeffnung auf Tunnel; der LiDAR sieht dann
    auch den freien Raum hinter und neben dem Bot, und "weit weg vom Eingang" wuerde
    genau dorthin ziehen, aus dem Tunnel weg.

    Geplant wird nur durch bekanntes Gebiet (allow_unknown: false); ein
    Frontier-Punkt liegt per Definition am Rand des Bekannten. Neu bewertet wird bei
    Ankunft oder nach stall_timeout ohne Ankunft.
    """

    def __init__(self):
        super().__init__('tunnel_explorer_node')

        # --- Parameter ---
        self.declare_parameter('recompute_rate', 1.0, frange(0.2, 10.0))
        self.declare_parameter('arrival_radius', 0.3, frange(0.05, 2.0))
        self.declare_parameter('stall_timeout', 10.0, frange(1.0, 60.0))
        self.declare_parameter('free_cost_threshold', 50, irange(1, 99))
        # In Zellen der global_costmap, haengt also an deren resolution.
        self.declare_parameter('min_frontier_cluster_cells', 6, irange(1, 1000))
        self.declare_parameter('entrance_weight', 1.0, frange(0.0, 10.0))
        self.declare_parameter('robot_distance_weight', 0.3, frange(0.0, 10.0))
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('output_path_topic', '/tunnel/explore_path')
        self._param_names = [
            'arrival_radius', 'stall_timeout', 'free_cost_threshold',
            'min_frontier_cluster_cells', 'entrance_weight', 'robot_distance_weight',
            'odom_frame', 'robot_frame',
        ]

        # --- Subscriber ---
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)
        self.sub_entrance_pose = self.create_subscription(
            PoseStamped, '/tunnel/entrance_pose', self.cb_entrance_pose, 1)
        self.sub_costmap = self.create_subscription(
            OccupancyGrid, self.get_parameter('costmap_topic').value, self.cb_costmap, 1)

        # --- Publisher ---
        self.pub_explore_path = self.create_publisher(
            Path, self.get_parameter('output_path_topic').value, 1)
        self.plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # --- Timer ---
        self.timer = self.create_timer(
            1.0 / self.get_parameter('recompute_rate').value, self.cb_timer)

        # --- Zustand ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.control_mode = None
        self.entrance_pose = None       # (x, y, yaw) im odom-Frame, yaw = Einfahrtrichtung
        self.latest_costmap = None
        self.current_target = None      # (x, y) im odom-Frame
        self.target_set_time = None
        self.planning_in_progress = False
        self._pending_target = None

        self.get_logger().info('Started tunnel_explorer_node Node.')

    # --- Callbacks ---

    def cb_control_mode(self, msg):
        new_mode = control_mode_from_msg(self, msg)
        if new_mode is None:
            return
        if new_mode != ControlType.Tunnel and self.control_mode == ControlType.Tunnel:
            # Eingang und Ziele gehoeren zu diesem Tunnel; der naechste bringt eigene.
            self.entrance_pose = None
            self.current_target = None
            self.target_set_time = None
            self._pending_target = None
        self.control_mode = new_mode

    def cb_entrance_pose(self, pose):
        self.entrance_pose = (pose.pose.position.x, pose.pose.position.y,
                              yaw_from_quaternion(pose.pose.orientation))

    def cb_costmap(self, grid):
        self.latest_costmap = grid

    def cb_timer(self):
        if self.control_mode != ControlType.Tunnel:
            return
        if self.latest_costmap is None or self.entrance_pose is None:
            return
        if self.planning_in_progress:
            return
        read_params(self, self._param_names)

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return

        need_new_target = self.current_target is None
        if not need_new_target:
            dist_to_target = math.hypot(
                robot_pose[0] - self.current_target[0], robot_pose[1] - self.current_target[1])
            stalled = ((self.get_clock().now() - self.target_set_time).nanoseconds / 1e9
                       >= self.stall_timeout)
            need_new_target = dist_to_target <= self.arrival_radius or stalled

        if need_new_target:
            self._choose_new_target(robot_pose)

    def cb_plan_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(
                'Explorations-Pfad wurde vom planner_server abgelehnt.', throttle_duration_sec=5.0)
            self.planning_in_progress = False
            return
        goal_handle.get_result_async().add_done_callback(self.cb_plan_result)

    def cb_plan_result(self, future):
        self.planning_in_progress = False

        # Der Modus kann waehrend der Planung gewechselt haben.
        if self.control_mode != ControlType.Tunnel or self._pending_target is None:
            return

        result = future.result()
        path = result.result.path if result is not None else None
        if path is None or not path.poses:
            self.get_logger().warn(
                'planner_server hat keinen Pfad zum gewaehlten Frontier-Punkt gefunden.',
                throttle_duration_sec=5.0)
            return

        self.pub_explore_path.publish(path)
        self.current_target = self._pending_target
        self.target_set_time = self.get_clock().now()
        self.get_logger().info(
            f'Neues Explorations-Ziel: x={self.current_target[0]:.2f} '
            f'y={self.current_target[1]:.2f} (odom).')

    # --- Hilfsmethoden ---

    def _lookup_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.robot_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF-Lookup fuer Roboterpose fehlgeschlagen: {e}', throttle_duration_sec=2.0)
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _choose_new_target(self, robot_pose):
        candidate = self._pick_frontier(self.latest_costmap, robot_pose, self.entrance_pose)
        if candidate is None:
            self.get_logger().warn(
                'Keine Frontier-Kandidaten gefunden -- Tunnel schon vollstaendig erkundet '
                'oder Karte noch zu leer.', throttle_duration_sec=5.0)
            return
        self._request_path(candidate)

    def _pick_frontier(self, grid_msg, robot_pose, entrance_pose):
        info = grid_msg.info
        h, w = info.height, info.width
        grid = np.asarray(grid_msg.data, dtype=np.int16).reshape(h, w)

        free = (grid >= 0) & (grid < self.free_cost_threshold)
        unknown = grid == -1

        padded_unknown = np.pad(unknown, 1, mode='constant', constant_values=False)
        neighbor_unknown = np.zeros_like(unknown)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                neighbor_unknown |= padded_unknown[1 + dr:1 + dr + h, 1 + dc:1 + dc + w]

        # Nur Zellen jenseits der Eingangslinie, also im Tunnel.
        ex, ey, eyaw = entrance_pose
        xs = info.origin.position.x + (np.arange(w) + 0.5) * info.resolution
        ys = info.origin.position.y + (np.arange(h) + 0.5) * info.resolution
        ahead = ((xs[None, :] - ex) * math.cos(eyaw) + (ys[:, None] - ey) * math.sin(eyaw)) > 0.0

        frontier = (free & neighbor_unknown & ahead).astype(np.uint8)
        if not np.any(frontier):
            return None

        num_labels, labels = cv2.connectedComponents(frontier, connectivity=8)

        best_score = None
        best_point = None
        for label in range(1, num_labels):
            rows, cols = np.where(labels == label)
            if rows.size < self.min_frontier_cluster_cells:
                continue

            world_x = info.origin.position.x + (float(np.mean(cols)) + 0.5) * info.resolution
            world_y = info.origin.position.y + (float(np.mean(rows)) + 0.5) * info.resolution
            dist_from_entrance = math.hypot(world_x - entrance_pose[0], world_y - entrance_pose[1])
            dist_from_robot = math.hypot(world_x - robot_pose[0], world_y - robot_pose[1])
            score = (self.entrance_weight * dist_from_entrance
                     - self.robot_distance_weight * dist_from_robot)

            if best_score is None or score > best_score:
                best_score = score
                best_point = (world_x, world_y)
        return best_point

    def _request_path(self, target_xy):
        if not self.plan_client.server_is_ready():
            self.get_logger().warn(
                'compute_path_to_pose Action-Server nicht erreichbar -- laeuft planner_server?',
                throttle_duration_sec=5.0)
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal.header.frame_id = self.odom_frame
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.pose.position.x = target_xy[0]
        goal_msg.goal.pose.position.y = target_xy[1]
        goal_msg.goal.pose.orientation.w = 1.0
        goal_msg.use_start = False

        self.planning_in_progress = True
        self._pending_target = target_xy
        self.plan_client.send_goal_async(goal_msg).add_done_callback(self.cb_plan_response)

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down tunnel_explorer_node Node.')


def main(args=None):
    run_node(tunnel_explorer_node, args)


if __name__ == '__main__':
    main()
