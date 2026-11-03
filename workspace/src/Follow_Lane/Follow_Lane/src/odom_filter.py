"""Odometrie ohne Messausreisser: odom -> base_footprint aus /joint_states und /imu.

Ein:  /joint_states, /imu, /tf (nur zur Pruefung, ob der Bot selbst noch sendet)
Aus:  TF odom -> base_footprint, /odom_filtered (Odometry)

Der turtlebot3_node liefert ab und zu EINE /joint_states-Nachricht mit Unsinn fuer beide
Raeder (gemessen 2026-11-03: -51674 rad statt 391 rad, eine Nachricht spaeter wieder
richtig, auch im Stand). Sein diff_drive_controller rechnet das als Fahrt von ~1700 m hin
und zurueck. Nav2s mitfahrende Costmaps verwerfen dabei alles, und es bleibt ein Versatz
von Metern.

Gerechnet wird wie im turtlebot3_node (odometry.use_imu: true): Weg aus den Raedern,
Drehung aus dem IMU-Gierwinkel. Verworfen wird jede Nachricht, deren Raddrehung seit der
letzten gueltigen schneller waere als max_wheel_speed; die naechste wird wieder gegen die
letzte gueltige gerechnet. Ebenso ein IMU-Sprung ueber max_yaw_rate, dann gilt die Drehung
aus den Raedern.

Voraussetzung: der Bot sendet die TF nicht selbst (bot-up.sh, BOT_ODOM_TF=false). Tut er
es doch, haette base_footprint zwei Eltern-Sender; dann sendet dieser Node keine TF und
meldet das.
"""
import math
import time
from collections import deque

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

from Follow_Lane.src.common import frange, irange, read_params, run_node, yaw_from_quaternion

# So lange (s) nach einer fremden odom-TF bleibt die eigene aus.
FOREIGN_TF_HOLD_S = 2.0
# Annahme fuer dt, wenn zwei Nachrichten denselben Zeitstempel tragen.
DEFAULT_DT_S = 0.05


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class odom_filter(Node):

    def __init__(self):
        super().__init__('odom_filter')

        # --- Parameter ---
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('left_joint', 'wheel_left_joint')
        self.declare_parameter('right_joint', 'wheel_right_joint')
        # Burger: wheels.radius / wheels.separation aus turtlebot3_bringup/param/burger.yaml
        self.declare_parameter('wheel_radius', 0.033, frange(0.01, 0.2))
        self.declare_parameter('wheel_separation', 0.160, frange(0.05, 1.0))
        # Burger faehrt hoechstens 0,22 m/s bzw. 2,84 rad/s; Reserve fuer Rauschen.
        self.declare_parameter('max_wheel_speed', 0.5, frange(0.05, 5.0))
        self.declare_parameter('max_yaw_rate', 5.0, frange(0.5, 20.0))
        # Bleiben so viele Nachrichten in Folge unplausibel, war es kein Ausreisser
        # (z.B. Zaehler im OpenCR zurueckgesetzt): neue Werte uebernehmen, ohne zu fahren.
        self.declare_parameter('resync_after', 10, irange(2, 1000))
        self.declare_parameter('use_imu', True)
        self._param_names = ['odom_frame', 'robot_frame', 'left_joint', 'right_joint',
                             'wheel_radius', 'wheel_separation', 'max_wheel_speed',
                             'max_yaw_rate', 'resync_after', 'use_imu']
        read_params(self, self._param_names)

        # --- Subscriber ---
        self.sub_joints = self.create_subscription(
            JointState, '/joint_states', self.cb_joint_states, qos_profile_sensor_data)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.cb_imu, qos_profile_sensor_data)
        self.sub_tf = self.create_subscription(TFMessage, '/tf', self.cb_tf, 100)

        # --- Publisher ---
        self.pub_odom = self.create_publisher(Odometry, '/odom_filtered', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Zustand ---
        self.pose = [0.0, 0.0, 0.0]      # x, y, yaw im odom_frame
        self._last_wheels = None         # (zeit_s, links_rad, rechts_rad) der letzten gueltigen
        self._imu_yaw = None             # neuester IMU-Gierwinkel
        self._last_imu_yaw = None        # IMU-Gierwinkel bei der letzten gueltigen Nachricht
        self._rejected = 0
        self._own_stamps = deque(maxlen=50)
        self._foreign_tf_time = None

        self.get_logger().info('Started odom_filter Node.')

    # --- Callbacks ---

    def cb_imu(self, msg):
        self._imu_yaw = yaw_from_quaternion(msg.orientation)

    def cb_tf(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id != self.odom_frame or tf.child_frame_id != self.robot_frame:
                continue
            stamp = (tf.header.stamp.sec, tf.header.stamp.nanosec)
            if stamp not in self._own_stamps:
                self._foreign_tf_time = time.monotonic()

    def cb_joint_states(self, msg):
        read_params(self, self._param_names)
        try:
            left = msg.position[msg.name.index(self.left_joint)]
            right = msg.position[msg.name.index(self.right_joint)]
        except (ValueError, IndexError):
            self.get_logger().warn(
                f'/joint_states ohne {self.left_joint}/{self.right_joint}: {list(msg.name)}',
                throttle_duration_sec=10.0)
            return
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        imu_yaw = self._imu_yaw if self.use_imu else None

        if self._last_wheels is None:
            self._accept(now, left, right, imu_yaw)
            return

        last_t, last_left, last_right = self._last_wheels
        dt = now - last_t if now > last_t else DEFAULT_DT_S
        d_left, d_right = left - last_left, right - last_right
        max_step = self.max_wheel_speed / self.wheel_radius * dt
        if not (math.isfinite(d_left) and math.isfinite(d_right)) \
                or max(abs(d_left), abs(d_right)) > max_step:
            self._rejected += 1
            if self._rejected < self.resync_after:
                self.get_logger().warn(
                    f'Radwerte verworfen: links {d_left:+.3f} rad, rechts {d_right:+.3f} rad '
                    f'in {dt * 1000:.0f} ms (erlaubt {max_step:.3f})', throttle_duration_sec=1.0)
                return
            self.get_logger().error(
                f'{self._rejected} Radwerte in Folge unplausibel -- uebernehme sie als neuen '
                'Bezug, ohne die Pose zu aendern.')
            self._accept(now, left, right, imu_yaw)
            return

        d_s = self.wheel_radius * (d_left + d_right) / 2.0
        d_yaw = self.wheel_radius * (d_right - d_left) / self.wheel_separation
        if imu_yaw is not None and self._last_imu_yaw is not None:
            d_imu = wrap(imu_yaw - self._last_imu_yaw)
            if abs(d_imu) <= self.max_yaw_rate * dt:
                d_yaw = d_imu
            else:
                self.get_logger().warn(
                    f'IMU-Sprung {math.degrees(d_imu):+.1f} Grad in {dt * 1000:.0f} ms verworfen, '
                    'Drehung aus den Raedern.', throttle_duration_sec=1.0)

        x, y, yaw = self.pose
        mid = yaw + d_yaw / 2.0
        self.pose = [x + d_s * math.cos(mid), y + d_s * math.sin(mid), wrap(yaw + d_yaw)]
        self._accept(now, left, right, imu_yaw)
        self._publish(msg.header.stamp, d_s / dt, d_yaw / dt)

    # --- Hilfsmethoden ---

    def _accept(self, now, left, right, imu_yaw):
        self._last_wheels = (now, left, right)
        self._last_imu_yaw = imu_yaw
        self._rejected = 0

    def _publish(self, stamp, v, w):
        x, y, yaw = self.pose
        q = Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.robot_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = v, w
        self.pub_odom.publish(odom)

        if self._foreign_tf_time is not None \
                and time.monotonic() - self._foreign_tf_time < FOREIGN_TF_HOLD_S:
            self.get_logger().error(
                'Der Bot sendet selbst odom -> base_footprint -- eigene TF bleibt aus, '
                'Ausreisser kommen weiter durch. Bot mit BOT_ODOM_TF=false hochfahren '
                '(bot-up.sh).', throttle_duration_sec=10.0)
            return
        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = self.robot_frame
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation = q
        self._own_stamps.append((stamp.sec, stamp.nanosec))
        self.tf_broadcaster.sendTransform(tf)

    def shut_down(self):
        self.get_logger().info('Shutting down odom_filter Node.')


def main(args=None):
    run_node(odom_filter, args)


if __name__ == '__main__':
    main()
