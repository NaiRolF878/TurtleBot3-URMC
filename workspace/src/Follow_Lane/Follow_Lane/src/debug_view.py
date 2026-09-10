
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
from std_msgs.msg import Float64, String, UInt8

from Follow_Lane.src.switch_control import ControlType



BATT_FULL_V = 12.6    # 4.2 V/Zelle
BATT_EMPTY_V = 11.0

# Farbe pro Fahrmodus (BGR), damit auf einen Blick erkennbar ist, wer gerade faehrt.
MODE_COLOR = {
    ControlType.Lane:     (135, 250, 159),   # gruen  -- PID faehrt
    ControlType.Obstacle: (0, 165, 255),     # orange -- avoid_node faehrt
    ControlType.Stop:     (60, 60, 255),     # rot
}

# Wer bei welchem Modus tatsaechlich /cmd_vel bespielt -- die haeufigste
# Verstaendnisfrage beim Testen ("warum lenkt der PID nicht mehr?").
MODE_DRIVER = {
    ControlType.Lane:     'PID',
    ControlType.Obstacle: 'avoid_node',
}


class debug_view(Node):
    def __init__(self):
        super().__init__('debug_view')

        self.get_logger().info('Started debug_view Node.')

        self.bridge = CvBridge()

        #Subscriber

        self.sub_follow_lane_debug = self.create_subscription(FollowLaneDebug, '/detect/lane_debug',self.cb_get_follow_lane_debug, 1)
        self.sub_battery_state = self.create_subscription(
            BatteryState, '/battery_state', self.cb_get_battery_state, qos_profile_sensor_data)
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_get_control_mode, 1)
        # Wer tatsaechlich faehrt -- der Mode sagt nur, wer fahren DUERFTE.
        self.sub_cmd_source = self.create_subscription(
            String, '/control/cmd_source', self.cb_get_cmd_source, 1)
        self.sub_avoid_offset = self.create_subscription(
            Float64, '/avoid/offset', self.cb_get_avoid_offset, 1)

        #Publisher
        self.pub_debug_image = self.create_publisher(Image,'/lokal/debug_view',1)

        #Variablen
        self.follow_lane_debug = None
        self.target_v = 5
        self.actual_v = 4.56

        self.battery_level = None

        self.control_mode = None
        self.cmd_source = None
        self.cmd_source_stamp = None
        self.avoid_offset = None

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

    def cb_get_control_mode(self, msg):
        try:
            self.control_mode = ControlType(msg.data)
        except ValueError:
            self.control_mode = None

    def cb_get_cmd_source(self, msg):
        self.cmd_source = msg.data
        self.cmd_source_stamp = self.get_clock().now()

    def cb_get_avoid_offset(self, msg):
        self.avoid_offset = msg.data

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

        def proj(x, y):
            p = np.array([[[float(x), float(y)]]], dtype=np.float32)
            r = cv2.perspectiveTransform(p, M_inv)
            return (int(r[0][0][0]), int(r[0][0][1]))

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

        # center_*_x/y kommen aus detect_lane vom entzerrten Ausschnitt (crop_img()),
        # nicht vom raw_image -- ohne Ruecktransformation ueber M_inv landen die
        # Punkte an der falschen Stelle (falscher Massstab + falsche Perspektive).
        size = self.follow_lane_debug.crop_im_size
        pts_warped = np.float32([[0, 0], [size, 0], [size, size], [0, size]])
        M_inv = cv2.getPerspectiveTransform(pts_warped, follow_lane_RoI.astype(np.float32))

        cv2.circle(raw, proj(self.follow_lane_debug.center_yellow_x, self.follow_lane_debug.center_yellow_y), 6, (0, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(raw, proj(self.follow_lane_debug.center_white_x, self.follow_lane_debug.center_white_y), 6, (255, 255, 255), -1, cv2.LINE_AA)
        lane_center_y = (self.follow_lane_debug.center_yellow_y + self.follow_lane_debug.center_white_y) / 2
        cv2.circle(raw, proj(self.follow_lane_debug.lane_center, lane_center_y), 6, (255, 255, 255), -1, cv2.LINE_AA)

        panel_w = 230

        panelGeneric = np.zeros((raw_h, panel_w, 3), dtype=np.uint8)
        cv2.line(panelGeneric,(panel_w-2,5),(panel_w-2,raw_h-5),thickness= 1 ,color=(90,90,90),lineType=cv2.LINE_AA)
        cv2.putText(panelGeneric,'TurtleBot'    ,(8,40) ,cv2.FONT_HERSHEY_SIMPLEX   ,1    ,(135, 250, 159)    ,2  ,cv2.LINE_AA)
        cv2.putText(panelGeneric,'Speed'        ,(8,80) ,cv2.FONT_HERSHEY_SIMPLEX   ,1    ,(200, 200, 200)    ,1  ,cv2.LINE_AA)
        cv2.putText(panelGeneric,'Target:'    ,(8,120) ,cv2.FONT_HERSHEY_SIMPLEX   ,0.55   ,(200, 200, 200)    ,1  ,cv2.LINE_AA)
        cv2.putText(panelGeneric,f"{self.target_v} m/s"    ,(8,155) ,cv2.FONT_HERSHEY_SIMPLEX   ,1    ,(135, 250, 159)    ,2  ,cv2.LINE_AA)
        cv2.putText(panelGeneric,'Actual:'    ,(8,195) ,cv2.FONT_HERSHEY_SIMPLEX   ,0.55    ,(200, 200, 200)    ,1  ,cv2.LINE_AA)
        cv2.putText(panelGeneric,f"{self.actual_v} m/s"    ,(8,230) ,cv2.FONT_HERSHEY_SIMPLEX   ,1    ,(250, 135, 223)    ,2  ,cv2.LINE_AA)
        

        panelMode = self.build_mode_panel(raw_h, panel_w)

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

    def build_mode_panel(self, height, width):
        panel = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.line(panel, (1, 5), (1, height-5), thickness=1, color=(90,90,90), lineType=cv2.LINE_AA)

        cv2.putText(panel, 'Mode', (8,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (200,200,200), 1, cv2.LINE_AA)

        if self.control_mode is None:
            cv2.putText(panel, '-- kein Mode --', (8,80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120,120,120), 1, cv2.LINE_AA)
            return panel

        color = MODE_COLOR.get(self.control_mode, (200,200,200))
        cv2.putText(panel, self.control_mode.name, (8,85),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, color, 2, cv2.LINE_AA)

        # Wer laut Modus fahren duerfte -- im Obstacle-Modus ist der PID aus.
        driver = MODE_DRIVER.get(self.control_mode, 'kein Regler')
        cv2.putText(panel, 'faehrt:', (8,125),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200,200,200), 1, cv2.LINE_AA)
        cv2.putText(panel, driver, (8,160),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)

        self.draw_cmd_source_status(panel)
        return panel

    # Farbe je Mux-Quelle, passend zum Mode-Farbschema darueber.
    SOURCE_COLOR = {
        'safe':  (60, 60, 255),      # rot    -- Not-Aus
        'avoid': (0, 165, 255),      # orange -- avoid_node
        'lane':  (135, 250, 159),    # gruen  -- PID
        'none':  (120, 120, 120),    # grau   -- Nullbefehl
    }

    def draw_cmd_source_status(self, panel):
        """Welche Quelle der Mux gerade durchlaesst, plus der Ausweich-Offset.

        Der Mode sagt nur, wer fahren duerfte -- der Not-Aus greift quer dazu
        durch, ohne den Mode zu aendern. Beim Testen ist genau das die Frage:
        "warum faehrt er nicht, obwohl Lane steht?"
        """
        cv2.putText(panel, 'Mux-Quelle:', (8,215),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200,200,200), 1, cv2.LINE_AA)

        if self.cmd_source_stamp is None:
            text, color = 'nie empfangen', (120,120,120)
        else:
            age = (self.get_clock().now() - self.cmd_source_stamp).nanoseconds / 1e9
            if age > 2.0:
                text, color = f'still ({age:.0f}s)', (60,60,255)
            else:
                text = self.cmd_source or '?'
                color = self.SOURCE_COLOR.get(self.cmd_source, (200,200,200))

        cv2.putText(panel, text, (8,250), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)

        # Offset: Vorzeichen als Richtungspfeil, damit es ohne Nachdenken lesbar
        # ist (positiv = nach links, wie der Spur-Error von detect_lane).
        cv2.putText(panel, 'Ausweich-Offset:', (8,290),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200,200,200), 1, cv2.LINE_AA)
        if self.avoid_offset is None:
            off_text, off_color = '--', (120,120,120)
        elif abs(self.avoid_offset) < 0.01:
            off_text, off_color = '0.00', (200,200,200)
        else:
            arrow = '<' if self.avoid_offset > 0 else '>'
            off_text = f'{arrow} {abs(self.avoid_offset):.2f}'
            off_color = (0,165,255)
        cv2.putText(panel, off_text, (8,325), cv2.FONT_HERSHEY_SIMPLEX, 0.8, off_color, 2, cv2.LINE_AA)

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

        if icon.shape[2] == 4:
            alpha = icon[:, :, 3:4].astype(np.float32) / 255.0
            head_panel[y:y+icon_h, x:x+icon_w] = (
                icon[:, :, :3] * alpha
                + head_panel[y:y+icon_h, x:x+icon_w] * (1.0 - alpha)
            ).astype(np.uint8)
        else:
            head_panel[y:y+icon_h, x:x+icon_w] = icon[:, :, :3]

    def show_battery_level(self):

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
            # rclpy raeumt den Context im eigenen SIGINT-Handler ab -- sonst wirft der zweite Aufruf
            if rclpy.ok():
                rclpy.shutdown()



if __name__ == '__main__':
        main()
