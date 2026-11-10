"""2D-Simulator fuer den TurtleBot3 Burger: faehrt nach /cmd_vel und liefert, was der Bot
selbst sendet. Die Pipeline (odom_filter, Nav2, switch_control, ...) laeuft unveraendert.

Ein:  /cmd_vel (Twist), /sim/reset (Empty: zurueck an den Start), /control/mode
Aus:  /joint_states, /imu, /scan (LDS-03), statische TF des Bots (base_footprint ->
      base_link -> base_scan, imu_link), /sim/pose (PoseStamped, Welt), /sim/view (Image,
      Draufsicht), /detect/lines_lost (Bool, Ersatz fuer die Kamera, siehe cb_mode)

Die odom-TF sendet odom_filter aus /joint_states und /imu, wie am Bot.
"""
import math
import os
import time

import cv2
import numpy as np

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, JointState, LaserScan
from std_msgs.msg import Bool, Empty, UInt8
from tf2_ros import StaticTransformBroadcaster

from urmc_sim.world import World

WORLD_FRAME = 'world'
ROBOT_FRAME = 'base_footprint'
# Burger (turtlebot3_burger.urdf, burger.yaml)
WHEEL_RADIUS = 0.033
WHEEL_SEPARATION = 0.160
BASE_LINK_Z = 0.010
SCAN_XYZ = (-0.032, 0.0, 0.172)          # base_link -> base_scan
IMU_XYZ = (-0.032, 0.0, 0.068)           # base_link -> imu_link
MAX_WHEEL_VEL = 0.22
MAX_ANG_VEL = 2.84
# Motoren folgen dem Befehl mit dieser Zeitkonstante (s); ohne Befehl so lange (s) halten sie an.
MOTOR_TAU_S = 0.1
CMD_TIMEOUT_S = 0.5
# Beruehrungen naeher als das (s) aneinander zaehlen als eine Kollision (Bot drueckt weiter).
CONTACT_HOLD_S = 0.5
# Umriss wie footprint in nav2_params.yaml (ROBOTIS: im Kreis r = 105 mm um die Radachse).
FOOTPRINT = np.array([
    [0.037, -0.069], [0.037, 0.069], [0.033, 0.089], [-0.033, 0.089], [-0.079, 0.069],
    [-0.092, 0.05], [-0.101, 0.028], [-0.101, -0.028], [-0.092, -0.05], [-0.079, -0.069],
    [-0.033, -0.089], [0.033, -0.089]])
# LDS-03: 0,9 Grad, 0,05 bis 12 m, 10 Hz
SCAN_BEAMS = 400
SCAN_RANGE_MIN = 0.05
SCAN_RANGE_MAX = 12.0
SCAN_HZ = 10.0
SCAN_HEIGHT = BASE_LINK_Z + SCAN_XYZ[2]
# ControlType.TunnelEntry (Follow_Lane/src/common.py)
TUNNEL_ENTRY = 7
PHYSICS_HZ = 50.0
JOINT_HZ = 30.0
IMU_HZ = 50.0
VIEW_HZ = 5.0
# Draufsicht: hoechstens so viele Pixel je Meter bzw. Pixel Kantenlaenge, Rand (m)
VIEW_SCALE = 400.0
VIEW_MAX_PX = 900
VIEW_MARGIN_M = 0.15


def quaternion_z(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class sim_node(Node):
    """Kinematik eines Differentialantriebs: Befehl auf 0,22 m/s bzw. 2,84 rad/s und die
    Radgrenze begrenzt, Motoren mit MOTOR_TAU_S. Wuerde der Umriss ein Objekt beruehren,
    bleibt der Bot stehen (Kollision im Log und rot in der Draufsicht). Der LiDAR sieht nur
    Objekte, die hoeher als seine Ebene sind; scan_noise_m streut jede Entfernung.
    """

    def __init__(self):
        super().__init__('urmc_sim')

        # --- Parameter ---
        self.declare_parameter('world', Parameter.Type.STRING)
        self.declare_parameter('scan_noise_m', Parameter.Type.DOUBLE)
        try:
            world = self.get_parameter('world').value
            self.scan_noise_m = self.get_parameter('scan_noise_m').value
        except ParameterUninitializedException as e:
            self.get_logger().fatal(f'Ohne Wert: {e} -- sim.yaml oder world fehlt.')
            raise SystemExit(1)
        if not os.path.isabs(world):
            world = os.path.join(get_package_share_directory('urmc_sim'), 'worlds',
                                 world if world.endswith('.yaml') else world + '.yaml')
        self.world = World.load(world)

        # --- Subscriber ---
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 10)
        self.create_subscription(Empty, '/sim/reset', self.cb_reset, 1)
        self.create_subscription(UInt8, '/control/mode', self.cb_mode, 1)

        # --- Publisher ---
        self.pub_joints = self.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
        self.pub_imu = self.create_publisher(Imu, '/imu', qos_profile_sensor_data)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.pub_pose = self.create_publisher(PoseStamped, '/sim/pose', 10)
        self.pub_view = self.create_publisher(Image, '/sim/view', 1)
        self.pub_lines = self.create_publisher(Bool, '/detect/lines_lost', 1)
        self.static_tf = StaticTransformBroadcaster(self)
        self.publish_static_tf()

        # --- Zustand ---
        self.bridge = CvBridge()
        self.rng = np.random.default_rng()
        self.pose = list(self.world.start)
        self.v = self.w = 0.0
        self.cmd = (0.0, 0.0)
        self.cmd_time = 0.0
        self.wheels = [0.0, 0.0]
        self.wheel_vel = [0.0, 0.0]
        self.collided = False
        self.collisions = 0
        self.contact_time = -math.inf
        self.last_ranges = None
        self.last_step = time.monotonic()
        self.mode = None

        # --- Timer ---
        self.create_timer(1.0 / PHYSICS_HZ, self.step)
        self.create_timer(1.0 / JOINT_HZ, self.publish_joints)
        self.create_timer(1.0 / IMU_HZ, self.publish_imu)
        self.create_timer(1.0 / SCAN_HZ, self.publish_scan)
        self.create_timer(1.0 / VIEW_HZ, self.publish_view)

        self.get_logger().info(f'Started urmc_sim, Welt {world}.')

    # --- Callbacks ---

    def cb_cmd_vel(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_time = time.monotonic()

    def cb_mode(self, msg):
        """Ohne Kamerabild: beim Wechsel auf TunnelEntry einmal 'Linien gesehen', wie
        detect_lane vor dem Tunnel. Sonst hielte tunnel_explorer_node den Bot schon fuer
        drinnen und liesse die Einfahrt durch die Tuer aus."""
        if msg.data == TUNNEL_ENTRY and self.mode != TUNNEL_ENTRY:
            self.pub_lines.publish(Bool(data=False))
        self.mode = msg.data

    def cb_reset(self, _msg):
        self.pose = list(self.world.start)
        self.v = self.w = 0.0
        self.collided = False
        self.contact_time = -math.inf
        self.get_logger().info('Zurueck an den Start (Welt). Odometrie laeuft weiter wie nach '
                               'einem Anheben -- im Not-Aus L druecken.')

    # --- Physik ---

    def step(self):
        now = time.monotonic()
        dt = min(now - self.last_step, 0.1)
        self.last_step = now
        v_cmd, w_cmd = self.cmd if now - self.cmd_time <= CMD_TIMEOUT_S else (0.0, 0.0)
        v_cmd = max(-MAX_WHEEL_VEL, min(MAX_WHEEL_VEL, v_cmd))
        w_cmd = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, w_cmd))
        outer = abs(v_cmd) + abs(w_cmd) * WHEEL_SEPARATION / 2.0
        if outer > MAX_WHEEL_VEL:
            v_cmd *= MAX_WHEEL_VEL / outer
            w_cmd *= MAX_WHEEL_VEL / outer
        k = min(1.0, dt / MOTOR_TAU_S)
        self.v += (v_cmd - self.v) * k
        self.w += (w_cmd - self.w) * k

        x, y, yaw = self.pose
        mid = yaw + self.w * dt / 2.0
        new = [x + self.v * math.cos(mid) * dt, y + self.v * math.sin(mid) * dt, yaw + self.w * dt]
        if self.world.collides(self.footprint(new)):
            if now - self.contact_time > CONTACT_HOLD_S:
                self.collisions += 1
                self.get_logger().warn(
                    f'Kollision {self.collisions} bei x={x:.3f} y={y:.3f} '
                    f'Winkel {math.degrees(yaw):.1f} Grad (v={self.v:+.3f}, w={self.w:+.2f}).')
            self.contact_time = now
            self.collided = True
            self.v = self.w = 0.0
            self.wheel_vel = [0.0, 0.0]
            return
        self.collided = now - self.contact_time <= CONTACT_HOLD_S
        self.pose = new
        left = (self.v - self.w * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
        right = (self.v + self.w * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
        self.wheel_vel = [left, right]
        self.wheels[0] += left * dt
        self.wheels[1] += right * dt

    def footprint(self, pose):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return FOOTPRINT @ np.array([[c, s], [-s, c]]) + np.array([x, y])

    # --- Ausgaben ---

    def publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['wheel_left_joint', 'wheel_right_joint']
        msg.position = [float(v) for v in self.wheels]
        msg.velocity = [float(v) for v in self.wheel_vel]
        self.pub_joints.publish(msg)

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        q = quaternion_z(self.pose[2] - self.world.start[2])
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        msg.angular_velocity.z = float(self.w)
        self.pub_imu.publish(msg)

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = WORLD_FRAME
        pose.pose.position.x, pose.pose.position.y = float(self.pose[0]), float(self.pose[1])
        (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
         pose.pose.orientation.w) = quaternion_z(self.pose[2])
        self.pub_pose.publish(pose)

    def publish_scan(self):
        x, y, yaw = self.pose
        ox = x + SCAN_XYZ[0] * math.cos(yaw) - SCAN_XYZ[1] * math.sin(yaw)
        oy = y + SCAN_XYZ[0] * math.sin(yaw) + SCAN_XYZ[1] * math.cos(yaw)
        inc = 2.0 * math.pi / SCAN_BEAMS
        local = np.arange(SCAN_BEAMS) * inc
        ranges = self.world.raycast((ox, oy), local + yaw, SCAN_RANGE_MAX, SCAN_HEIGHT)
        if self.scan_noise_m > 0.0:
            ranges = ranges + self.rng.normal(0.0, self.scan_noise_m, ranges.size)
        ranges = np.where(ranges >= SCAN_RANGE_MIN, ranges, np.inf)
        self.last_ranges = ((ox, oy, yaw), local, ranges)

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_scan'
        msg.angle_min = 0.0
        msg.angle_max = float(local[-1])
        msg.angle_increment = float(inc)
        msg.scan_time = float(1.0 / SCAN_HZ)
        msg.range_min = float(SCAN_RANGE_MIN)
        msg.range_max = float(SCAN_RANGE_MAX)
        msg.ranges = [float(r) for r in ranges]
        self.pub_scan.publish(msg)

    def publish_static_tf(self):
        def tf(parent, child, xyz):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
            t.transform.rotation.w = 1.0
            return t
        self.static_tf.sendTransform([
            tf(ROBOT_FRAME, 'base_link', (0.0, 0.0, BASE_LINK_Z)),
            tf('base_link', 'base_scan', SCAN_XYZ),
            tf('base_link', 'imu_link', IMU_XYZ),
        ])

    def publish_view(self):
        if self.pub_view.get_subscription_count() == 0:
            return
        xmin, ymin, xmax, ymax = self.world.bounds()
        xmin, ymin = xmin - VIEW_MARGIN_M, ymin - VIEW_MARGIN_M
        xmax, ymax = xmax + VIEW_MARGIN_M, ymax + VIEW_MARGIN_M
        scale = min(VIEW_SCALE, VIEW_MAX_PX / max(xmax - xmin, ymax - ymin))
        w = int((xmax - xmin) * scale)
        h = int((ymax - ymin) * scale)
        view = np.full((h, w, 3), 40, np.uint8)

        def px(pts):
            pts = np.atleast_2d(pts)
            return np.column_stack([(pts[:, 0] - xmin) * scale,
                                    (ymax - pts[:, 1]) * scale]).astype(np.int32)

        for b in self.world.boxes:
            color = (170, 170, 170) if b.height > SCAN_HEIGHT else (110, 110, 110)
            cv2.fillPoly(view, [px(b.corners)], color)
        for c in self.world.cylinders:
            color = (170, 170, 170) if c.height > SCAN_HEIGHT else (110, 110, 110)
            cv2.circle(view, tuple(px(c.center)[0]), max(1, int(c.radius * scale)), color, -1)
        if self.last_ranges is not None:
            (ox, oy, yaw), local, ranges = self.last_ranges
            ok = np.isfinite(ranges)
            a = local[ok] + yaw
            hits = np.column_stack([ox + ranges[ok] * np.cos(a), oy + ranges[ok] * np.sin(a)])
            for p in px(hits):
                cv2.circle(view, tuple(p), 2, (0, 0, 255), -1)
        color = (0, 0, 255) if self.collided else (0, 200, 0)
        cv2.polylines(view, [px(self.footprint(self.pose))], True, color, 2, cv2.LINE_AA)
        x, y, yaw = self.pose
        tip = (x + 0.08 * math.cos(yaw), y + 0.08 * math.sin(yaw))
        cv2.line(view, tuple(px((x, y))[0]), tuple(px(tip)[0]), color, 2, cv2.LINE_AA)
        cv2.putText(view, f'x {x:+.3f}  y {y:+.3f}  {math.degrees(yaw):+.1f} Grad  '
                    f'Kollisionen {self.collisions}', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 1, cv2.LINE_AA)
        msg = self.bridge.cv2_to_imgmsg(view, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_view.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = sim_node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
