import math

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from std_msgs.msg import UInt8
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from Follow_Lane.src.switch_control import ControlType


class tunnel_explorer_node(Node):
    """Sucht im Tunnel den Ausgang: Frontier-Zellen auf der global_costmap
    (frei, direkt neben Unbekanntem), bewertet nach "weit weg vom Eingang" und
    "schnell erreichbar". Zum besten Punkt laesst der planner_server einen Pfad
    rechnen, der auf /tunnel/explore_path geht.

    Geplant wird nur durch bereits bekanntes Gebiet (allow_unknown: false) --
    ein Frontier-Punkt liegt per Definition am Rand des Bekannten.

    Neu bewertet bei Ankunft oder nach stall_timeout ohne Fortschritt.
    """

    def __init__(self):
        super().__init__('tunnel_explorer_node')

        self.get_logger().info('Started tunnel_explorer_node Node.')

        def frange(lo, hi):
            return ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=lo, to_value=hi, step=0.0)])

        self.declare_parameter('recompute_rate', 1.0, frange(0.2, 10.0))
        self.declare_parameter('arrival_radius', 0.3, frange(0.05, 2.0))
        self.declare_parameter('stall_timeout', 10.0, frange(1.0, 60.0))
        self.declare_parameter('free_cost_threshold', 50,
            ParameterDescriptor(integer_range=[IntegerRange(from_value=1, to_value=99, step=1)]))
        self.declare_parameter('min_frontier_cluster_cells', 6,
            ParameterDescriptor(integer_range=[IntegerRange(from_value=1, to_value=1000, step=1)]))
        self.declare_parameter('entrance_weight', 1.0, frange(0.0, 10.0))
        self.declare_parameter('robot_distance_weight', 0.3, frange(0.0, 10.0))
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('output_path_topic', '/tunnel/explore_path')

        self._param_names = [
            'arrival_radius', 'stall_timeout', 'free_cost_threshold',
            'min_frontier_cluster_cells', 'entrance_weight', 'robot_distance_weight',
            'frame_id', 'robot_frame',
        ]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cbGetControlMode, 1)
        self.sub_entrance_pose = self.create_subscription(
            PoseStamped, '/tunnel/entrance_pose', self.cbEntrancePose, 1)
        costmap_topic = self.get_parameter('costmap_topic').value
        self.sub_costmap = self.create_subscription(
            OccupancyGrid, costmap_topic, self.cbCostmap, 1)

        output_path_topic = self.get_parameter('output_path_topic').value
        self.pub_explore_path = self.create_publisher(Path, output_path_topic, 1)

        self._plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        self.current_control_mode = ControlType.Lane
        self.entrance_pose = None       # (x, y) im odom-Frame
        self.latest_costmap = None      # nav_msgs/OccupancyGrid
        self.current_target = None      # (x, y) im odom-Frame
        self.target_set_time = None
        self.planning_in_progress = False
        self._pending_target = None

        recompute_rate = self.get_parameter('recompute_rate').value
        self.timer = self.create_timer(1.0 / recompute_rate, self.cbTimer)

    def cbGetControlMode(self, msg):
        try:
            new_mode = ControlType(msg.data)
        except ValueError:
            self.get_logger().warn(f'Unbekannter Control-Mode empfangen: {msg.data}')
            return

        if new_mode != ControlType.Tunnel and self.current_control_mode == ControlType.Tunnel:
            self.current_target = None
            self.target_set_time = None

        self.current_control_mode = new_mode

    def cbEntrancePose(self, pose):
        self.entrance_pose = (pose.pose.position.x, pose.pose.position.y)

    def cbCostmap(self, grid):
        self.latest_costmap = grid

    def cbTimer(self):
        if self.current_control_mode != ControlType.Tunnel:
            return
        if self.latest_costmap is None or self.entrance_pose is None:
            return
        if self.planning_in_progress:
            return

        for name in self._param_names:
            setattr(self, name, self.get_parameter(name).value)

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return

        need_new_target = self.current_target is None
        if not need_new_target:
            dist_to_target = math.hypot(
                robot_pose[0] - self.current_target[0], robot_pose[1] - self.current_target[1])
            now = self.get_clock().now()
            stalled = (now - self.target_set_time).nanoseconds / 1e9 >= self.stall_timeout
            need_new_target = dist_to_target <= self.arrival_radius or stalled

        if need_new_target:
            self._choose_new_target(robot_pose)

    def _lookup_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.frame_id, self.robot_frame, rclpy.time.Time())
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
        self._request_path(robot_pose, candidate)

    def _pick_frontier(self, grid_msg, robot_pose, entrance_pose):
        info = grid_msg.info
        h, w = info.height, info.width
        grid = np.asarray(grid_msg.data, dtype=np.int16).reshape(h, w)

        free = (grid >= 0) & (grid < self.free_cost_threshold)
        unknown = (grid == -1)

        padded_unknown = np.pad(unknown, 1, mode='constant', constant_values=False)
        neighbor_unknown = np.zeros_like(unknown)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                neighbor_unknown |= padded_unknown[1 + dr:1 + dr + h, 1 + dc:1 + dc + w]

        frontier = (free & neighbor_unknown).astype(np.uint8)
        if not np.any(frontier):
            return None

        num_labels, labels = cv2.connectedComponents(frontier, connectivity=8)

        best_score = None
        best_point = None
        for label in range(1, num_labels):
            rows, cols = np.where(labels == label)
            if rows.size < self.min_frontier_cluster_cells:
                continue

            centroid_row = float(np.mean(rows))
            centroid_col = float(np.mean(cols))
            world_x = info.origin.position.x + (centroid_col + 0.5) * info.resolution
            world_y = info.origin.position.y + (centroid_row + 0.5) * info.resolution

            dist_from_entrance = math.hypot(world_x - entrance_pose[0], world_y - entrance_pose[1])
            dist_from_robot = math.hypot(world_x - robot_pose[0], world_y - robot_pose[1])
            score = self.entrance_weight * dist_from_entrance - self.robot_distance_weight * dist_from_robot

            if best_score is None or score > best_score:
                best_score = score
                best_point = (world_x, world_y)

        return best_point

    def _request_path(self, robot_pose, target_xy):
        if not self._plan_client.server_is_ready():
            self.get_logger().warn(
                'compute_path_to_pose Action-Server nicht erreichbar -- laeuft planner_server?',
                throttle_duration_sec=5.0)
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal.header.frame_id = self.frame_id
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.pose.position.x = target_xy[0]
        goal_msg.goal.pose.position.y = target_xy[1]
        goal_msg.goal.pose.orientation.w = 1.0
        goal_msg.use_start = False

        self.planning_in_progress = True
        self._pending_target = target_xy
        send_future = self._plan_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._cb_plan_response)

    def _cb_plan_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(
                'Explorations-Pfad wurde vom planner_server abgelehnt.', throttle_duration_sec=5.0)
            self.planning_in_progress = False
            return

        goal_handle.get_result_async().add_done_callback(self._cb_plan_result)

    def _cb_plan_result(self, future):
        self.planning_in_progress = False

        # Modus kann waehrend der Planung gewechselt haben
        if self.current_control_mode != ControlType.Tunnel or self._pending_target is None:
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
            f'Neues Explorations-Ziel: x={self.current_target[0]:.2f} y={self.current_target[1]:.2f} (odom).')


def main(args=None):
    rclpy.init(args=args)
    node = tunnel_explorer_node()
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
