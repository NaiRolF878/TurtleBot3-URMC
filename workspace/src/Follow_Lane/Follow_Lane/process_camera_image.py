import numpy as np
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, FloatingPointRange

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class process_camera_image(Node):
    def __init__(self):
        super.__init__('process_camera_image')

        #Parameter:
        # Wiederverwendbare Ranges
        hsv  = ParameterDescriptor(integer_range=[IntegerRange(from_value=0,    to_value=255,  step=1)])
        crop = ParameterDescriptor(integer_range=[IntegerRange(from_value=-100, to_value=1000, step=1)])

        # --- HSV-Parameter ---
        # --- White ---
        self.declare_parameter('hue_white_l',        0,   hsv)
        self.declare_parameter('hue_white_h',        255, hsv)
        self.declare_parameter('saturation_white_l', 0,   hsv)
        self.declare_parameter('saturation_white_h', 75,  hsv)
        self.declare_parameter('lightness_white_l',  161, hsv)
        self.declare_parameter('lightness_white_h',  255, hsv)

        # --- Yellow ---
        self.declare_parameter('hue_yellow_l',        13,  hsv)
        self.declare_parameter('hue_yellow_h',        60,  hsv)
        self.declare_parameter('saturation_yellow_l', 60,  hsv)
        self.declare_parameter('saturation_yellow_h', 255, hsv)
        self.declare_parameter('lightness_yellow_l',  120, hsv)
        self.declare_parameter('lightness_yellow_h',  255, hsv)

        # --- Crop-Image-Parameter ---
        # --- Perspektiv-Punkte ---
        # --- (Im Moment noch DuckieCam Parameter) ---
        self.declare_parameter('top_left_x',     159, crop)
        self.declare_parameter('top_left_y',     218, crop)
        self.declare_parameter('top_right_x',    441, crop)
        self.declare_parameter('top_right_y',    218, crop)
        self.declare_parameter('bottom_left_x',  606, crop)
        self.declare_parameter('bottom_left_y',  382, crop)
        self.declare_parameter('bottom_right_x', -29, crop)
        self.declare_parameter('bottom_right_y', 382, crop)

        # --- Sonstige ---
        self.declare_parameter('detection_row_factor', 0.75,
            ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=0.5, to_value=0.95, step=0.01)]))
        self.declare_parameter('min_lane_width', 80,
            ParameterDescriptor(integer_range=[IntegerRange(from_value=0, to_value=300, step=1)]))




        #Subscriber:

        self.image_sub = self.create_subscription(Image, '/camera/image_projected', self.cbGetRawImage ,10)

        #Publisher:


        #Andere Variablen:

        self._crop_im_size = 400
        self.raw_image = None

    def cbGetRawImage(self,img):
        self.raw_image = img

        self.cbFindLane(img)


    def cbFindLane(img):
        null


    def crop_img(self,img):
        
        pts1 = np.float32([
            [self.top_left_x,     self.top_left_y],
            [self.top_right_x,    self.top_right_y],
            [self.bottom_right_x, self.bottom_right_y],
            [self.bottom_left_x,  self.bottom_left_y],])
        
        pts2 = np.float32([[0,0],[self._crop_im_size,0],[0,self._crop_im_size],[self._crop_im_size,self._crop_im_size]])

        M = cv2.getPerspectiveTransform(pts1,pts2)
        self.M = M
        return cv2.warpPerspective(img,M,(self._crop_im_size,self._crop_im_size))

    


    



