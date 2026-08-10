import rclpy
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

        self.current_control_mode = ControlType.Lane

        #--- Subscriber ---  


        #--- Publisher ---
        self.pub_control = self.create_publisher(UInt8,'/control/mode',1)



    def main(args=None):
        rclpy.init(args=args)
        node = switch_control()

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:

            node.destroy_node()
            rclpy.shutdown()



    if __name__ == '__main__':
        main()


