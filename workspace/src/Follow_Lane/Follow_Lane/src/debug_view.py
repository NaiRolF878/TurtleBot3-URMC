
import numpy as np
import cv2


from rclpy.node import Node
from follow_lane_msg.msg import FollowLaneDebug

class debug_view(Node):
    def __init__(self):
        super.__init__('debug_view')


        #Subscriber

        self.sub_follow_lane_debug = self.create_subscription(FollowLaneDebug, '/detect/lane_debug',self.cb_get_follow_lane_debug, 1)


        #Variablen
        self.follow_lane_debug = FollowLaneDebug


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

        cv2.polylines(raw,[follow_lane_RoI],isClosed=True, color=(0, 255, 255), thickness=2)


        panelGeneric = np.zeros((raw_h, 230, 3), dtype=np.uint8)
        cv2.putText(panelGeneric,'Speed',(5,20),cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)


        panelMode = np.zeros((raw_h, 230, 3), dtype=np.uint8)



        final_debug_image = np.hstack([raw, panelGeneric, panelMode])
