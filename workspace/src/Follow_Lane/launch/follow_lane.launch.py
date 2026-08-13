from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='Follow_Lane', executable='ingress_node',    name='ingress_node'),
        Node(package='Follow_Lane', executable='detect_lane',     name='detect_lane'),
        Node(
            package='Follow_Lane', executable='control_driving', name='control_driving',
            # control_driving publiziert bewusst auf /control/cmd_vel -- das ist der
            # Ausgang des Reglers, nicht zwingend der Eingang des Roboters. Der
            # turtlebot3_node aus dem Bringup hoert aber ausschliesslich auf /cmd_vel.
            # Ohne dieses Remapping laufen die Twist-Nachrichten ins Leere: der Regler
            # rechnet korrekt, "ros2 topic echo /control/cmd_vel" zeigt Werte, aber
            # "Subscription count" bleibt 0 und der Bot steht.
            # Bewusst hier statt im Code: so bleibt der Regler wiederverwendbar (z.B.
            # in Gazebo oder hinter einem Mux, der mehrere Regler arbitriert).
            remappings=[('/control/cmd_vel', '/cmd_vel')],
        ),
        Node(package='Follow_Lane', executable='switch_control',  name='switch_control'),
        Node(package='Follow_Lane', executable='debug_view',      name='debug_view'),
        Node(package='Follow_Lane', executable='safety_stop',     name='safety_stop'),
    ])
