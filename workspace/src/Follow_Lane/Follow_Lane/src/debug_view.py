
import math
import os

import numpy as np
import cv2
import datetime as dt

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from follow_lane_msg.msg import FollowLaneDebug
from sensor_msgs.msg import Image, BatteryState



BATT_FULL_V = 12.6    # 4.2 V/Zelle
BATT_EMPTY_V = 11.0


class debug_view(Node):
    def __init__(self):
        super().__init__('debug_view')

        self.get_logger().info('Started debug_view Node.')

        self.bridge = CvBridge()

        #Subscriber

        self.sub_follow_lane_debug = self.create_subscription(FollowLaneDebug, '/detect/lane_debug',self.cb_get_follow_lane_debug, 1)
        self.sub_battery_state = self.create_subscription(
            BatteryState, '/battery_state', self.cb_get_battery_state, qos_profile_sensor_data)

        #Publisher
        self.pub_debug_image = self.create_publisher(Image,'/lokal/debug_view',1)

        #Variablen
        self.follow_lane_debug = None
        self.target_v = 5
        self.actual_v = 4.56

        self.battery_level = None

        media_dir = os.path.join(get_package_share_directory('Follow_Lane'), 'media', 'Battery')
        self.battery_icons = []
        for name in ('battery_0.png', 'battery_1.png', 'battery_2.png', 'battery_3.png'):
            path = os.path.join(media_dir, name)
            icon = cv2.imread(path, cv2.IMREAD_UNCHANGED)
            if icon is None:
                self.get_logger().warn(f'Batterie-Icon nicht gefunden: {path}')
            self.battery_icons.append(icon)

    def cb_get_follow_lane_debug(self, follow_lane_debug):
        self.follow_lane_debug = follow_lane_debug
        self.build_debug_image()

    def cb_get_battery_state(self, state):
        self.battery_level = self.battery_percent(state)

    def battery_percent(self, state):
        if not math.isnan(state.percentage) and state.percentage > 0.0:
            pct = state.percentage
            return int(round(pct if pct > 1.0 else pct * 100))

        if math.isnan(state.voltage) or state.voltage <= 0.0:
            return None
        ratio = (state.voltage - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V)
        return int(round(max(0.0, min(1.0, ratio)) * 100))


    def build_debug_image(self):
        if self.follow_lane_debug is None:
            return

        if not self.follow_lane_debug.raw_image.data:
            self.get_logger().warn('Debug-Message enthaelt kein Bild.', throttle_duration_sec=5.0)
            return

        try:
            raw = self.bridge.compressed_imgmsg_to_cv2(
                self.follow_lane_debug.raw_image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Debug-Bild konnte nicht dekodiert werden: {e}')
            return

        raw_h, raw_w = raw.shape[:2]

        follow_lane_RoI = np.array([
            [self.follow_lane_debug.top_left_x,     self.follow_lane_debug.top_left_y],
            [self.follow_lane_debug.top_right_x,    self.follow_lane_debug.top_right_y],
            [self.follow_lane_debug.bottom_right_x, self.follow_lane_debug.bottom_right_y],
            [self.follow_lane_debug.bottom_left_x,  self.follow_lane_debug.bottom_left_y],
        ], np.int32)

        cv2.polylines(raw,[follow_lane_RoI],isClosed=True, color=(0, 255, 255), thickness=2, lineType=cv2.LINE_AA)

        panel_w = 230

        panelGeneric = np.zeros((raw_h, panel_w, 3), dtype=np.uint8)
        cv2.line(panelGeneric,(panel_w-2,5),(panel_w-2,raw_h-5),thickness= 1 ,color=(90,90,90),lineType=cv2.LINE_AA)
        cv2.putText(panelGeneric,'TurtleBot'    ,(8,40) ,cv2.FONT_HERSHEY_SIMPLEX   ,1    ,(135, 250, 159)    ,2  ,cv2.LINE_AA)
        cv2.putText(panelGeneric,'Speed'        ,(8,80) ,cv2.FONT_HERSHEY_SIMPLEX   ,1    ,(200, 200, 200)    ,1  ,cv2.LINE_AA)
        cv2.putText(panelGeneric,'Target:'    ,(8,120) ,cv2.FONT_HERSHEY_SIMPLEX   ,0.55   ,(200, 200, 200)    ,1  ,cv2.LINE_AA)
        cv2.putText(panelGeneric,f"{self.target_v} m/s"    ,(8,155) ,cv2.FONT_HERSHEY_SIMPLEX   ,1    ,(135, 250, 159)    ,2  ,cv2.LINE_AA)
        cv2.putText(panelGeneric,'Actual:'    ,(8,195) ,cv2.FONT_HERSHEY_SIMPLEX   ,0.55    ,(200, 200, 200)    ,1  ,cv2.LINE_AA)
        cv2.putText(panelGeneric,f"{self.actual_v} m/s"    ,(8,230) ,cv2.FONT_HERSHEY_SIMPLEX   ,1    ,(250, 135, 223)    ,2  ,cv2.LINE_AA)

        panelMode = np.zeros((raw_h, panel_w, 3), dtype=np.uint8)

        total_w = raw_w + panel_w * 2
        head_panel = np.full((40, total_w, 3), (220,220,220) ,dtype=np.uint8)
        now = dt.datetime.now()
        cv2.putText(head_panel,now.strftime("%d.%m.%Y"),(10,28),cv2.FONT_HERSHEY_SIMPLEX   ,1    ,(0,0,0)    ,2  ,cv2.LINE_AA)
        cv2.putText(head_panel,now.strftime("%H:%M"),(200,28),cv2.FONT_HERSHEY_SIMPLEX   ,1    ,(0,0,0)    ,2  ,cv2.LINE_AA)

        self.draw_battery(head_panel, total_w)

        final_debug_image = np.hstack([raw, panelGeneric, panelMode])
        final_debug_image = np.vstack([head_panel,final_debug_image])

        msg = self.bridge.cv2_to_imgmsg(final_debug_image, encoding='bgr8')
        msg.header = self.follow_lane_debug.raw_image.header
        self.pub_debug_image.publish(msg)
        return final_debug_image

    def draw_battery(self, head_panel, total_w):
        text = '--%' if self.battery_level is None else f"{self.battery_level}%"
        (text_w, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
        text_x = max(0, total_w - text_w - 10)
        cv2.putText(head_panel, text, (text_x, 33), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2, cv2.LINE_AA)

        icon = self.show_battery_level()
        if icon is None:
            return

        icon_h, icon_w = icon.shape[:2]
        x = text_x - icon_w - 8
        y = (head_panel.shape[0] - icon_h) // 2
        if x < 0 or y < 0 or icon_h > head_panel.shape[0]:
            return

        # Bewusst per numpy statt cv2.copyTo: dessen Ziel waere hier ein nicht
        # zusammenhaengender Teilausschnitt, in den OpenCV je nach Version nicht
        # zurueckschreibt -- das Icon fehlt dann kommentarlos.
        if icon.shape[2] == 4:
            alpha = icon[:, :, 3:4].astype(np.float32) / 255.0
            head_panel[y:y+icon_h, x:x+icon_w] = (
                icon[:, :, :3] * alpha
                + head_panel[y:y+icon_h, x:x+icon_w] * (1.0 - alpha)
            ).astype(np.uint8)
        else:
            head_panel[y:y+icon_h, x:x+icon_w] = icon[:, :, :3]

    def show_battery_level(self):
        # None -> kein Icon. Ohne diese Zeile wuerde der Vergleich unten mit
        # "'>=' not supported between NoneType and int" fliegen, bevor die erste
        # Batterie-Nachricht da ist -- und damit das ganze Debug-Bild verhindern.
        if self.battery_level is None:
            return None
        if   self.battery_level >= 75: return self.battery_icons[3]
        elif self.battery_level >= 50: return self.battery_icons[2]
        elif self.battery_level >= 25: return self.battery_icons[1]
        else:             return self.battery_icons[0]


def main(args=None):
        rclpy.init(args=args)
        node = debug_view()

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:

            node.destroy_node()
            rclpy.shutdown()



if __name__ == '__main__':
        main()
