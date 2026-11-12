"""Odometrie ohne Messausreisser: odom -> base_footprint aus /joint_states und /imu.

Ein:  /joint_states, /imu, /tf (nur zur Pruefung, ob der Bot selbst noch sendet)
Aus:  TF odom -> base_footprint, /odom_filtered (Odometry)

Der turtlebot3_node liefert ab und zu eine /joint_states-Nachricht mit Unsinn fuer beide
Raeder (z.B. -51674 rad statt 391 rad, auch im Stand). Daraus wuerde eine Fahrt von
~1700 m, die mitfahrenden Costmaps verloeren alles, und es bliebe ein Versatz von Metern.

Gerechnet wird wie im turtlebot3_node (odometry.use_imu: true): Weg aus den Raedern,
Drehung aus dem IMU-Gierwinkel. Verworfen wird jede Nachricht, deren Raddrehung seit der
letzten gueltigen schneller waere als MAX_WHEEL_SPEED; die naechste wird wieder gegen die
letzte gueltige gerechnet. Ebenso ein IMU-Sprung ueber MAX_YAW_RATE, dann gilt die Drehung
aus den Raedern.

IMU-Versatz (Lauf 11.11.: Kreisel meldete im Stand -0,70 bzw. +0,11 rad/s, die Odometrie
drehte weg, Zonen und Drehungen lagen daneben): Stehen beide Raeder STILL_MIN_MSGS Nachrichten
lang still, ist die echte Drehung 0 -- was die IMU dann meldet, ist ihr Versatz. Er wird
gemittelt und beim Fahren abgezogen; im Stand gilt Drehung 0. Kein Vergleich IMU <-> Raeder
je Nachricht: IMU und Raeder kommen nicht zeitgleich, der Vergleich schlug bei fast jeder
Drehung an, die dann aus den (beim Drehen rutschenden) Raedern kam -- Drehungen und Zonen lagen
daneben (Lauf 11.11.).

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

from Follow_Lane.src.common import run_node, yaw_from_quaternion

ODOM_FRAME = 'odom'
ROBOT_FRAME = 'base_footprint'
LEFT_JOINT = 'wheel_left_joint'
RIGHT_JOINT = 'wheel_right_joint'
# Burger: wheels.radius / wheels.separation aus turtlebot3_bringup/param/burger.yaml
WHEEL_RADIUS = 0.033
WHEEL_SEPARATION = 0.160
# Burger faehrt hoechstens 0,22 m/s bzw. 2,84 rad/s; Reserve fuer Rauschen.
MAX_WHEEL_SPEED = 0.5
MAX_YAW_RATE = 5.0
# Bleiben so viele Nachrichten in Folge unplausibel, war es kein Ausreisser (z.B. Zaehler im
# OpenCR zurueckgesetzt): neue Werte uebernehmen, ohne zu fahren.
RESYNC_AFTER = 10
# So lange (s) nach einer fremden odom-TF bleibt die eigene aus.
FOREIGN_TF_HOLD_S = 2.0
# Annahme fuer dt, wenn zwei Nachrichten denselben Zeitstempel tragen.
DEFAULT_DT_S = 0.05
# Stillstand: beide Raeder je Nachricht hoechstens so viel (rad), so viele Nachrichten in Folge
# (~0,5 s bei 18 Hz). Dann wird der IMU-Versatz gelernt (gleitender Mittelwert, Anteil
# BIAS_ALPHA je Nachricht; erster Wert direkt).
STILL_WHEEL_RAD = 0.002
STILL_MIN_MSGS = 9
BIAS_ALPHA = 0.3
# Gelernt wird nur aus einem ruhigen Fenster: so viele Nachrichten (~1 s) im Stand, Streuung der
# IMU-Rate hoechstens BIAS_MAX_STD (rad/s). Ein echter Versatz ist konstant; wird der Bot von Hand
# gedreht (Raeder stehen), schwankt die Rate und es wird nichts gelernt.
BIAS_WINDOW = 18
BIAS_MAX_STD = 0.03
# Gelernten Versatz ins Log, wenn er sich um so viel (rad/s) geaendert hat.
BIAS_LOG_STEP = 0.02


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class odom_filter(Node):

    def __init__(self):
        super().__init__('odom_filter')

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
        self._still = 0                  # Nachrichten in Folge mit stehenden Raedern
        self._bias = None                # gelernter IMU-Versatz (rad/s)
        self._bias_logged = None
        self._still_rates = deque(maxlen=BIAS_WINDOW)
        self._own_stamps = deque(maxlen=50)
        self._foreign_tf_time = None

        self.get_logger().info('Started odom_filter Node.')

    # --- Callbacks ---

    def cb_imu(self, msg):
        self._imu_yaw = yaw_from_quaternion(msg.orientation)

    def cb_tf(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id != ODOM_FRAME or tf.child_frame_id != ROBOT_FRAME:
                continue
            stamp = (tf.header.stamp.sec, tf.header.stamp.nanosec)
            if stamp not in self._own_stamps:
                self._foreign_tf_time = time.monotonic()

    def cb_joint_states(self, msg):
        try:
            left = msg.position[msg.name.index(LEFT_JOINT)]
            right = msg.position[msg.name.index(RIGHT_JOINT)]
        except (ValueError, IndexError):
            self.get_logger().warn(
                f'/joint_states ohne {LEFT_JOINT}/{RIGHT_JOINT}: {list(msg.name)}',
                throttle_duration_sec=10.0)
            return
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        imu_yaw = self._imu_yaw

        if self._last_wheels is None:
            self._accept(now, left, right, imu_yaw)
            return

        last_t, last_left, last_right = self._last_wheels
        dt = now - last_t if now > last_t else DEFAULT_DT_S
        d_left, d_right = left - last_left, right - last_right
        max_step = MAX_WHEEL_SPEED / WHEEL_RADIUS * dt
        if not (math.isfinite(d_left) and math.isfinite(d_right)) \
                or max(abs(d_left), abs(d_right)) > max_step:
            self._rejected += 1
            if self._rejected < RESYNC_AFTER:
                self.get_logger().warn(
                    f'Radwerte verworfen: links {d_left:+.3f} rad, rechts {d_right:+.3f} rad '
                    f'in {dt * 1000:.0f} ms (erlaubt {max_step:.3f})', throttle_duration_sec=1.0)
                return
            self.get_logger().error(
                f'{self._rejected} Radwerte in Folge unplausibel -- uebernehme sie als neuen '
                'Bezug, ohne die Pose zu aendern.')
            self._accept(now, left, right, imu_yaw)
            return

        d_s = WHEEL_RADIUS * (d_left + d_right) / 2.0
        d_wheel = WHEEL_RADIUS * (d_right - d_left) / WHEEL_SEPARATION
        d_yaw = self._yaw_step(d_left, d_right, d_wheel, imu_yaw, dt)

        x, y, yaw = self.pose
        mid = yaw + d_yaw / 2.0
        self.pose = [x + d_s * math.cos(mid), y + d_s * math.sin(mid), wrap(yaw + d_yaw)]
        self._accept(now, left, right, imu_yaw)
        self._publish(msg.header.stamp, d_s / dt, d_yaw / dt)

    # --- Hilfsmethoden ---

    def _yaw_step(self, d_left, d_right, d_wheel, imu_yaw, dt):
        """Drehung (rad) seit der letzten gueltigen Nachricht: IMU minus gelernter Versatz; im
        Stand 0; ohne IMU oder bei einem IMU-Sprung aus den Raedern."""
        still = max(abs(d_left), abs(d_right)) <= STILL_WHEEL_RAD
        self._still = self._still + 1 if still else 0
        if not still:
            self._still_rates.clear()
        if imu_yaw is None or self._last_imu_yaw is None:
            return 0.0 if still else d_wheel
        d_imu = wrap(imu_yaw - self._last_imu_yaw)
        if abs(d_imu) > MAX_YAW_RATE * dt:
            self.get_logger().warn(
                f'IMU-Sprung {math.degrees(d_imu):+.1f} Grad in {dt * 1000:.0f} ms verworfen, '
                'Drehung aus den Raedern.', throttle_duration_sec=1.0)
            return d_wheel
        if self._still >= STILL_MIN_MSGS:
            # Raeder stehen: echte Drehung 0, was die IMU meldet, ist ihr Versatz.
            self._still_rates.append(d_imu / dt)
            if len(self._still_rates) == BIAS_WINDOW:
                rates = list(self._still_rates)
                mean = sum(rates) / len(rates)
                std = math.sqrt(sum((r - mean) ** 2 for r in rates) / len(rates))
                if std <= BIAS_MAX_STD:
                    self._bias = mean if self._bias is None \
                        else self._bias + BIAS_ALPHA * (mean - self._bias)
                    self._log_bias()
            return 0.0
        if still:
            return 0.0
        return d_imu - (self._bias or 0.0) * dt

    def _log_bias(self):
        if self._bias_logged is None or abs(self._bias - self._bias_logged) >= BIAS_LOG_STEP:
            self._bias_logged = self._bias
            self.get_logger().info(
                f'IMU-Versatz im Stand gelernt: {self._bias:+.3f} rad/s '
                f'({math.degrees(self._bias):+.1f} Grad/s), wird abgezogen.')

    def _accept(self, now, left, right, imu_yaw):
        self._last_wheels = (now, left, right)
        self._last_imu_yaw = imu_yaw
        self._rejected = 0

    def _publish(self, stamp, v, w):
        x, y, yaw = self.pose
        q = Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = ROBOT_FRAME
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
        tf.child_frame_id = ROBOT_FRAME
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
