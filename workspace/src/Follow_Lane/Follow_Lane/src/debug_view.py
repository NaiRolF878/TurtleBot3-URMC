
import numpy as np
import cv2
import datetime as dt

import rclpy
from rclpy.node import Node
from follow_lane_msg.msg import FollowLaneDebug

class debug_view(Node):
    def __init__(self):
        super().__init__('debug_view')


        #Subscriber

        self.sub_follow_lane_debug = self.create_subscription(FollowLaneDebug, '/detect/lane_debug',self.cb_get_follow_lane_debug, 1)


        #Variablen
        self.follow_lane_debug = FollowLaneDebug
        self.target_v = 5
        self.actual_v = 4.56
        self.battery_level = 69
        

    def cb_get_follow_lane_debug(self,follow_lane_debug): self.follow_lane_debug = follow_lane_debug

    def build_debug_image(self):
        if self.follow_lane_debug == None and 1 == None:
            return

        raw = self.follow_lane_debug.raw_image

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

        head_panel = np.full((40,panel_w*2+raw_w,3), (220,220,220) ,dtype=np.uint8)
        now = dt.datetime.now()
        cv2.putText(head_panel,now.strftime("%d.%m.%Y"),(10,28),cv2.FONT_HERSHEY_SIMPLEX   ,1    ,(0,0,0)    ,2  ,cv2.LINE_AA)
        cv2.putText(head_panel,now.strftime("%H:%M"),(200,28),cv2.FONT_HERSHEY_SIMPLEX   ,1    ,(0,0,0)    ,2  ,cv2.LINE_AA)
        icon = self.show_battery_level()
        icon_h, icon_w = icon.shape[:2]

        cv2.putText(head_panel,f"{self.battery_level}%",(1020,33),cv2.FONT_HERSHEY_SIMPLEX   ,1    ,(0,0,0)    ,2  ,cv2.LINE_AA)
        x, y = 1000, (head_panel.shape[0] - icon_h) // 2
        roi = head_panel[y:y+icon_h, x:x+icon_w]
        cv2.copyTo(icon[:, :, :3], icon[:, :, 3], roi)


        final_debug_image = np.hstack([raw, panelGeneric, panelMode])
        final_debug_image = np.vstack([head_panel,final_debug_image])
        return final_debug_image;

    def show_battery_level(self):            # Beispiel-Schwellen fuer 3S-LiPo, an deinen Akku anpassen
        if   self.battery_level >= 75: return cv2.imread("Follow_Lane/media/Battery/battery_3.png", cv2.IMREAD_UNCHANGED)
        elif self.battery_level >= 50: return cv2.imread("Follow_Lane/media/Battery/battery_2.png", cv2.IMREAD_UNCHANGED)
        elif self.battery_level >= 25: return cv2.imread("Follow_Lane/media/Battery/battery_1.png", cv2.IMREAD_UNCHANGED)
        else:             return cv2.imread("Follow_Lane/media/Battery/battery_0.png", cv2.IMREAD_UNCHANGED)


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
    