from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from rclpy.node import Node

import rclpy

class control_driving(Node):
    def __init__(self):
        super.__init__('control_driving')


        # ---------------------------------
        # Parameter:
        # ---------------------------------

        # --- Wiederverwendbare Ranges ---
        def frange(lo, hi, step):
            return ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=lo, to_value=hi, step=step)])

        # --- PID / Speed ---
        self.declare_parameter('kp',                 5.5,  frange(0.0, 10.0, 0.01))
        self.declare_parameter('ki',                 0.15, frange(0.0, 10.0, 0.01))
        self.declare_parameter('kd',                 2.75, frange(0.0, 10.0, 0.01))
        self.declare_parameter('MAX_VEL',            0.34, frange(0.0, 10.0, 0.01))
        self.declare_parameter('speed_curve_factor', 1.0,  frange(0.0, 5.0,  0.01))
        self.declare_parameter('min_speed_factor',   0.5,  frange(0.0, 1.0,  0.01))


        #Subscriber
        self.sub_lane_error = self.create_subscription(Float64, '/detect/lane','cbFollowLane' ,10)


        #Publishers
        self.pub_cmd_vel = self.create_publisher(Twist,'/control/cmd_vel',1)

        #Andere Variablen
        self.lastError = 0
        self.integral = 0
        self.v = 0
        self.a = 0

    def cbFollowLane(self,error):
        if self._control_mode != ControlType.Lane:
            return

        Current_error = error.data

        #PID-Regler eingefügt
        self.integral += Current_error
        self.integral = max(-1.0, min(1.0, self.integral))  # ← Clamp
        p = self.kp * Current_error
        i = self.ki * self.integral
        d = self.kd * ((Current_error - self.lastError)/0.1) #soll wert gewichtung des D-Teils erhöhen, da die Funktion 10 mal pro Sekunde aufgerufen wird
        self.lastError = Current_error

        self.v = self.MAX_VEL * max(self.min_speed_factor, 1 - abs(error) * self.speed_curve_factor)
        self.a = p + i + d

        #v und Omega werte in Twist publishen
        twist = Twist()
        twist.linear.x = self.v
        twist.angular.z =  self.a
        #von online turtlebot3_autorace
        #twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        self.pub_cmd_vel.publish(twist)

    def shut_down(self):
        self.get_logger().info('Shuting down control_driving Node')
        twist = Twist()
        self.pub_cmd_vel.publish(twist)

def main(args= None):
    rclpy.Rate(1).sleep(5)
    rclpy.init(args=args)
    node = control_driving()
    try:
        None
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.shutdown

if __name__ == '__main__':
    main()


