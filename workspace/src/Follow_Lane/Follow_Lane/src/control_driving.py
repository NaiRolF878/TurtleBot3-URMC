from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, UInt8
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from rclpy.node import Node
from Follow_Lane.src.switch_control import ControlType

import signal
import time

import rclpy
from rclpy.signals import SignalHandlerOptions

class control_driving(Node):
    def __init__(self):
        super().__init__('control_driving')

        self.get_logger().info('Started control_driving Node.')

        # ---------------------------------
        # Parameter:
        # ---------------------------------

        # --- Wiederverwendbare Ranges ---
        # step=0.0: rclpy prueft sonst (wert-from) % step == 0, was mit Fliesskomma zufaellig fehlschlaegt
        def frange(lo, hi):
            return ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=lo, to_value=hi, step=0.0)])

        # --- PID / Speed ---
        self.declare_parameter('kp',                 5.5,  frange(0.0, 20.0))
        self.declare_parameter('ki',                 0.15, frange(0.0, 10.0))
        self.declare_parameter('kd',                 2.75, frange(0.0, 10.0))
        self.declare_parameter('MAX_VEL',            0.34, frange(0.0, 10.0))
        self.declare_parameter('speed_curve_factor', 1.0,  frange(0.0, 5.0))
        self.declare_parameter('min_speed_factor',   0.5,  frange(0.0, 10.0))

        self._param_names = [
            'kp', 'ki', 'kd', 'MAX_VEL', 'speed_curve_factor', 'min_speed_factor',
        ]


        #Subscriber
        self.sub_lane_error = self.create_subscription(Float64, '/detect/lane',self.cbFollowLane ,10)
        self.sub_control_mode = self.create_subscription(UInt8,'/control/mode',self.cbGetControlMode,1)


        #Publishers
        self.pub_cmd_vel = self.create_publisher(Twist,'/control/cmd_vel',1)

        #Andere Variablen
        # Default passend zum Default von switch_control, damit schon vor der ersten
        # /control/mode-Nachricht gefahren wird.
        self.current_control_mode = ControlType.Lane
        self.lastError = 0
        self.integral = 0
        self.v = 0
        self.a = 0

    def cbFollowLane(self,error):
        if self.current_control_mode != ControlType.Lane:
            return

        # declare_parameter legt keine self.<name>-Attribute an -> hier einlesen.
        # Gleichzeitig wirken so zur Laufzeit geaenderte Parameter sofort.
        for name in self._param_names:
            setattr(self, name, self.get_parameter(name).value)

        Current_error = error.data

        #PID-Regler eingefügt
        self.integral += Current_error
        self.integral = max(-1.0, min(1.0, self.integral))  # ← Clamp
        p = self.kp * Current_error
        i = self.ki * self.integral
        d = self.kd * ((Current_error - self.lastError)/0.1) #soll wert gewichtung des D-Teils erhöhen, da die Funktion 10 mal pro Sekunde aufgerufen wird
        self.lastError = Current_error

        self.v = self.MAX_VEL * max(self.min_speed_factor, 1 - abs(Current_error) * self.speed_curve_factor)
        self.a = p + i + d

        #v und Omega werte in Twist publishen
        twist = Twist()
        twist.linear.x = float(self.v)
        # Ungebremste PID-Ausgabe auf echter Hardware begrenzen.
        twist.angular.z = float(max(-2.0, min(2.0, self.a)))
        self.pub_cmd_vel.publish(twist)

    def cbGetControlMode(self,mode):
        # mode ist eine UInt8-Message; fuer den Vergleich mit ControlType.Lane
        # muss daraus das Enum-Member werden.
        try:
            new_mode = ControlType(mode.data)
        except ValueError:
            self.get_logger().warn(f'Unbekannter Control-Mode empfangen: {mode.data}')
            return

        # Beim Zurueckschalten auf Lane den Regler zuruecksetzen: lastError stammt
        # sonst von vor dem Ausflug (Sekunden alt) und der D-Anteil erzeugt sofort
        # einen Lenkschlag bis in die +/-2.0-Begrenzung.
        if new_mode == ControlType.Lane and self.current_control_mode != ControlType.Lane:
            self.integral = 0
            self.lastError = 0

        self.current_control_mode = new_mode


    def shut_down(self):
        self.get_logger().info('Shuting down control_driving Node')
        twist = Twist()
        for _ in range(10):
            self.pub_cmd_vel.publish(twist)
            time.sleep(0.05)

def main(args= None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = control_driving()

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
        node.shut_down()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()


