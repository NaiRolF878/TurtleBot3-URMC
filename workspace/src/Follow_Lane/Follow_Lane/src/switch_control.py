import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import UInt8

class ControlType(Enum):
    Lane = 1
    Obstacle = 2
    Stop = 3
    Intersection = 4
    Align = 5

class switch_control(Node):
    def __init__(self):
        super().__init__('switch_control')

        self.get_logger().info('Started switch_control Node.')

        self.current_control_mode = ControlType.Lane

        #--- Subscriber ---  


        #--- Publisher ---
        self.pub_control = self.create_publisher(UInt8,'/control/mode',1)

        # Ohne Timer wurde der Mode nie gesendet -> control_driving hat nie einen
        # Mode empfangen. Zyklisch senden, damit auch spaeter startende Nodes ihn bekommen.
        self.timer = self.create_timer(0.5, self.cbPubControlMode)

    def cbPubControlMode(self):
        msg = UInt8()
        msg.data = self.current_control_mode.value
        self.pub_control.publish(msg)



def main(args=None):
    # SIGINT selbst behandeln (KeyboardInterrupt), damit der Context beim Aufraeumen noch
    # gilt -- sonst bei Strg+C ExternalShutdownException-/"context is invalid"-Tracebacks.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = switch_control()

    try:
        # spin_once mit Timeout statt spin(): spin() wartet in C auf die naechste Nachricht,
        # Strg+C kaeme ohne Daten nie an (Knoten haengt beim Beenden).
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:

        node.destroy_node()
        # rclpy raeumt den Context im eigenen SIGINT-Handler ab -- sonst wirft der zweite Aufruf
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()


