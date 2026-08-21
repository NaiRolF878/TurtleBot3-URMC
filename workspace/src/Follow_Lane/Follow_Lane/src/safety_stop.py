import signal

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange


class safety_stop(Node):
    def __init__(self):
        super().__init__('safety_stop')

        self.get_logger().info('Started safety_stop Node.')

        # step=0.0: rclpy prueft sonst (wert-from) % step == 0, was mit Fliesskomma zufaellig fehlschlaegt
        self.declare_parameter(
            'timeout', 0.5,
            ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=0.1, to_value=5.0, step=0.0)]))

        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cbCmdVel, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)

        self.last_cmd_time = None
        self.is_stopped = True

        self.timer = self.create_timer(0.05, self.cbCheck)

    def cbCmdVel(self, twist):
        if twist.linear.x == 0.0 and twist.angular.z == 0.0:
            return
        self.last_cmd_time = self.get_clock().now()
        self.is_stopped = False

    def cbCheck(self):
        if self.is_stopped or self.last_cmd_time is None:
            return

        timeout = self.get_parameter('timeout').value
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if age < timeout:
            return

        self.get_logger().warn(
            f'Seit {age:.2f}s kein Fahrbefehl mehr -> Not-Aus.')
        self.stop_robot()
        self.is_stopped = True

    def stop_robot(self):
        twist = Twist()
        for _ in range(5):
            self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = safety_stop()

    stop = False

    def on_signal(signum, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
