"""Not-Aus / Fahrfreigabe zwischen Regler und Roboter.

    control_driving --/control/cmd_vel--> [notaus] --/cmd_vel--> turtlebot3_node

* Startet IMMER im Zustand STOPP -- der Bot faehrt erst nach Freigabe.
* Freigabe/Stopp umschalten:
    - Leertaste im Terminal dieses Nodes (q / Strg+C = beenden, mit Stopp)
    - rqt_reconfigure: Parameter `drive` (Haekchen) des Nodes /notaus
    - ros2 service call /notaus/set std_srvs/srv/SetBool "{data: true}"   (false = Stopp)
* Bei STOPP wird laufend Nullgeschwindigkeit gesendet (der turtlebot3_node haelt sonst den
  letzten Befehl). Bei FAHREN werden die Befehle des Reglers durchgereicht; kommt laenger als
  `cmd_timeout_s` nichts mehr vom Regler, wird ebenfalls Null gesendet.
* Zustand auf /notaus/drive (std_msgs/Bool, latched) fuer GUIs/andere Nodes.
* Beim Beenden: mehrfach Stopp. Danach kommt nichts mehr zum Bot durch (kein Remapping mehr).
"""
import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

STOP_RATE_HZ = 10.0


class notaus(Node):
    def __init__(self):
        super().__init__('notaus')
        # Bewusst False als Default -- auch wenn eine Parameterdatei etwas anderes sagt,
        # wird unten erzwungen, dass der Node im STOPP startet.
        self.declare_parameter('drive', False)
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.set_parameters([Parameter('drive', Parameter.Type.BOOL, False)])
        self.drive = False
        self.last_cmd_time = 0.0

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_state = self.create_publisher(Bool, '/notaus/drive', latched)
        self.create_subscription(Twist, '/control/cmd_vel', self.cb_cmd, 1)
        self.create_service(SetBool, '/notaus/set', self.cb_srv)
        self.add_on_set_parameters_callback(self.cb_params)
        self.create_timer(1.0 / STOP_RATE_HZ, self.tick)

        self.last_cmd_time = 0.0
        self.publish_state()
        self.get_logger().info('Not-Aus gestartet -- STOPP. Leertaste = Fahrt freigeben/stoppen.')

    # --- Zustand -------------------------------------------------------------------------
    def set_drive(self, drive, source, from_param=False):
        if drive == self.drive:
            return
        self.drive = drive
        if not drive:
            self.send_stop(5)
        self.last_cmd_time = 0.0          # nach Freigabe erst auf frische Reglerbefehle warten
        if not from_param:
            # Parameter mitziehen, damit rqt den richtigen Zustand zeigt (cb_params ist dann
            # ein No-op, weil self.drive schon stimmt)
            self.set_parameters([Parameter('drive', Parameter.Type.BOOL, drive)])
        self.publish_state()
        self.get_logger().warn(f'{"FAHREN freigegeben" if drive else "STOPP"} ({source})')
        print_state(drive)

    def publish_state(self):
        self.pub_state.publish(Bool(data=self.drive))

    def send_stop(self, n=1):
        for _ in range(n):
            self.pub_cmd.publish(Twist())

    # --- Callbacks -----------------------------------------------------------------------
    def cb_cmd(self, msg):
        self.last_cmd_time = time.monotonic()
        if self.drive:
            self.pub_cmd.publish(msg)

    def tick(self):
        timeout = self.get_parameter('cmd_timeout_s').value
        if not self.drive or time.monotonic() - self.last_cmd_time > timeout:
            self.send_stop()

    def cb_srv(self, req, res):
        self.set_drive(bool(req.data), 'Service')
        res.success, res.message = True, 'FAHREN' if self.drive else 'STOPP'
        return res

    def cb_params(self, params):
        for p in params:
            if p.name == 'drive' and bool(p.value) != self.drive:
                self.set_drive(bool(p.value), 'rqt/Parameter', from_param=True)
        return SetParametersResult(successful=True)


def print_state(drive):
    if drive:
        sys.stdout.write('\r\033[K\033[1;42;30m  >> FAHREN  \033[0m   Leertaste = STOPP, q = beenden ')
    else:
        sys.stdout.write('\r\033[K\033[1;41;37m  || STOPP   \033[0m   Leertaste = FAHREN, q = beenden ')
    sys.stdout.flush()


def keyboard_loop(node, stop_event):
    """Liest Einzeltasten aus dem Terminal (ohne Enter)."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        print_state(node.drive)
        while not stop_event.is_set():
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not r:
                continue
            ch = os.read(fd, 1)
            if ch == b' ':
                node.set_drive(not node.drive, 'Leertaste')
            elif ch in (b'q', b'Q', b'\x03'):
                stop_event.set()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        sys.stdout.write('\n')


def main(args=None):
    rclpy.init(args=args)
    node = notaus()
    stop_event = threading.Event()
    kb = None
    if sys.stdin.isatty():
        kb = threading.Thread(target=keyboard_loop, args=(node, stop_event), daemon=True)
        kb.start()
    else:
        node.get_logger().warn('Kein Terminal -- Leertaste geht nicht. Steuerung per rqt '
                               '(Parameter drive) oder /notaus/set.')
    try:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        node.drive = False
        node.send_stop(10)
        node.publish_state()
        if kb is not None:
            kb.join(timeout=1.0)
        node.get_logger().info('Not-Aus beendet -- Stopp gesendet.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
