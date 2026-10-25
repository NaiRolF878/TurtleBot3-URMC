#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gpiozero import OutputDevice


class magnet_controll(Node):
    def __init__(self):
        super().__init__('magnet_controll')
        self.magnet_active = False
        self.output_device = OutputDevice(17)

        self.sub_magnet_controll = self.create_subscription(Bool, '/lokal/magnet_control', self.cbControlMagnet, 1)

        self.pub_magnet_status = self.create_publisher(Bool, '/lokal/magnet_status', 1)

        self.get_logger().info('Magnet-Node gestartet: GPIO 17, Magnet AUS')
        self.get_logger().info('Hoert auf /lokal/magnet_control, meldet auf /lokal/magnet_status')

    def cbControlMagnet(self, magnet_on):
        self.get_logger().info(f'Befehl empfangen: {"AN" if magnet_on.data else "AUS"}')
        if magnet_on.data:
            self.output_device.on()
            self.magnet_active = True
        else:
            self.output_device.off()
            self.magnet_active = False

        self.pub_magnet_status.publish(Bool(data=self.magnet_active))
        self.get_logger().info(f'Magnet ist jetzt {"AN" if self.magnet_active else "AUS"} '
                               f'(GPIO 17 = {self.output_device.value})')


def main(args=None):
    rclpy.init(args=args)
    node = magnet_controll()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Strg+C -- beende Magnet-Node')
    finally:
        # Magnet beim Beenden immer ausschalten
        node.output_device.off()
        node.output_device.close()
        print('Magnet AUS, GPIO 17 freigegeben')
        node.destroy_node()
        # rclpy raeumt den Context im eigenen SIGINT-Handler ab -- sonst wirft der zweite Aufruf
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
