import numpy as np
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, FloatingPointRange

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image

from follow_lane_msg.msg import FollowLaneDebug


class detect_lane(Node):
    def __init__(self):
        super.__init__('detect_lane')

        # ---------------------------------
        # Parameter:
        # ---------------------------------

        # --- Wiederverwendbare Ranges ---
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
        self.declare_parameter('bottom_left_x',  -29, crop)
        self.declare_parameter('bottom_left_y',  382, crop)
        self.declare_parameter('bottom_right_x', 606, crop)
        self.declare_parameter('bottom_right_y', 382, crop)

        # --- Sonstige ---
        self.declare_parameter('detection_row_factor', 0.75,
            ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=0.5, to_value=0.95, step=0.01)]))
        self.declare_parameter('min_lane_width', 80,
            ParameterDescriptor(integer_range=[IntegerRange(from_value=0, to_value=300, step=1)]))

        

        #Subscriber:    
        self.sub_raw_image = self.create_subscription(Image, '/camera/image_projected', self.cbGetRawImage ,10)

        #Publisher:
        self.pub_lane_error = self.create_publisher(Float64, '/detect/lane',1)
        self.pub_debug_image = self.create_publisher(FollowLaneDebug, '/detect/lane_debug', 1)

        #Andere Variablen:

        self.min_lane_width = None  #Anpassen

        self.is_running = False

        self._crop_im_size = 400
        self.raw_image = None

    def cbGetRawImage(self,img):
        self.raw_image = img

        self.cbFindLane(img)


    def cbFindLane(self, img):
        if self.is_running:
            return
        self.is_running = True
        # Alle oben deklarierten Parameter als self.<name>-Attribute verfügbar machen
        param_names = [
            'hue_white_l', 'hue_white_h', 'saturation_white_l', 'saturation_white_h',
            'lightness_white_l', 'lightness_white_h',
            'hue_yellow_l', 'hue_yellow_h', 'saturation_yellow_l', 'saturation_yellow_h',
            'lightness_yellow_l', 'lightness_yellow_h',
            'top_left_x', 'top_left_y', 'top_right_x', 'top_right_y',
            'bottom_left_x', 'bottom_left_y', 'bottom_right_x', 'bottom_right_y',
            'detection_row_factor', 'min_lane_width',
        ]
        for name in param_names:
            setattr(self, name, self.get_parameter(name).value)

       
        detection_row_factor = self.detection_row_factor


        try:
            np_arr = np.frombuffer(img.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.raw_img = cv_image
            img = self.crop_img(cv_image)

            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            mask_yellow = cv2.inRange(hsv,
                               (self.hue_yellow_l,self.saturation_yellow_l, self.lightness_yellow_l),
                               (self.hue_yellow_h,self.saturation_yellow_h, self.lightness_yellow_h),)

            mask_white = cv2.inRange(hsv,
                               (self.hue_white_l,self.saturation_white_l, self.lightness_white_l),
                               (self.hue_white_h,self.saturation_white_h, self.lightness_white_h),)

            

            white_alternative = int(len(img[0]) * 0.95)
            yellow_alternative = int(len(img[0]) * 0.05)

            # Gelb fix am konfigurierten detection_row_factor
            yellow_row_factor = self.detection_row_factor
            center_yellow_y = int(len(img)*yellow_row_factor)
            center_yellow_x = self.get_x_for_driving(mask_yellow, center_yellow_y, yellow_alternative, left_line=True)

            # Weiße Maske: alles links von gelb + min_lane_width ausblenden
            mask_white_filtered = mask_white.copy()
            corridor_start = max(0, int(center_yellow_x) + self.min_lane_width)
            mask_white_filtered[:, :corridor_start] = 0

            # Weiß sucht weiter unten falls nicht gefunden
            center_white_y = int(len(img)*detection_row_factor)
            center_white_x = self.get_x_for_driving(mask_white_filtered, center_white_y, white_alternative, left_line=False)

            while center_white_x == white_alternative and detection_row_factor <= 0.95:
                detection_row_factor += 0.05
                center_white_y = int(len(img)*detection_row_factor)
                center_white_x = self.get_x_for_driving(mask_white_filtered, center_white_y, white_alternative, left_line=False)

            

            if center_white_x <= center_yellow_x:
                if center_white_x > int(len(img[0]) * 0.4):
                    center_yellow_x = yellow_alternative
                else:
                    center_white_x = white_alternative

            lane_center = (center_white_x + center_yellow_x) / 2
            lane_error = 1-(lane_center / len(img) * 2)

            msg_error = Float64()
            msg_error.data = lane_error

            self.pub_lane.publish(msg_error)
            self.pub_debug_follow_lane(center_yellow_x, center_yellow_y, center_white_x, center_white_y, lane_error, img)

        except Exception as e:
            self.get_logger().error(f"cbFindLane failed: {e}")
        finally:
            self.is_running = False

    def pub_debug_follow_lane(self, center_yellow_x, center_yellow_y, center_white_x, center_white_y, lane_error, img):
        follow_lane_debug_info = FollowLaneDebug()
        follow_lane_debug_info.center_white_x = center_white_x
        follow_lane_debug_info.center_white_y = center_white_y
        follow_lane_debug_info.center_yellow_x = center_yellow_x
        follow_lane_debug_info.center_yellow_y = center_yellow_y
        follow_lane_debug_info.lane_error = lane_error

        success, encoded = cv2.imencode('.jpg', img)
        if success:
            follow_lane_debug_info.raw_image.format = 'jpeg'
            follow_lane_debug_info.raw_image.data = encoded.tobytes()

        follow_lane_debug_info.top_left_x     = self.top_left_x
        follow_lane_debug_info.top_left_y     = self.top_left_y
        follow_lane_debug_info.top_right_x    = self.top_right_x
        follow_lane_debug_info.top_right_y    = self.top_right_y
        follow_lane_debug_info.bottom_left_x  = self.bottom_left_x
        follow_lane_debug_info.bottom_left_y  = self.bottom_left_y
        follow_lane_debug_info.bottom_right_x = self.bottom_right_x
        follow_lane_debug_info.bottom_right_y = self.bottom_right_y

        self.pub_debug_image.publish(follow_lane_debug_info)

    def crop_img(self,img):
        
        pts1 = np.float32([
            [self.top_left_x,     self.top_left_y],
            [self.top_right_x,    self.top_right_y],
            [self.bottom_right_x, self.bottom_right_y],
            [self.bottom_left_x,  self.bottom_left_y],])
        
        pts2 = np.float32([[0,0],[self._crop_im_size,0],[self._crop_im_size,self._crop_im_size],[0,self._crop_im_size]])

        M = cv2.getPerspectiveTransform(pts1,pts2)
        self.M = M
        return cv2.warpPerspective(img,M,(self._crop_im_size,self._crop_im_size))

    def get_x_for_driving(self, mask, distance, no_lane_value, left_line):
        grad = cv2.Sobel(mask, cv2.CV_16S, 1, 0, ksize=3, scale=1, delta=0, borderType=cv2.BORDER_DEFAULT)
        abs_grad = np.absolute(grad)
        uint8_grad = np.uint8(abs_grad)

        _,th1 = cv2.threshold(grad,127,255,cv2.THRESH_BINARY)

        a = []
        for row in range(max(0, distance-50), min(len(mask), distance+50)):
            if np.where(th1[row] == 255)[0].size == 0:
                continue
            else:
                if left_line:
                    a.append(np.where(th1[row] == 255)[0][-1])
                else:
                    a.append(np.where(th1[row] == 255)[0][0])

        if len(a) > 10:
            return np.median(a)
        else:
            return no_lane_value

    
    #NOCHMAL Überarbeiten
    def main(args=None):    
        rclpy.init(args=args)
        node = detect_lane()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.shut_down()
            node.destroy_node()
            rclpy.shutdown()


    if __name__ == '__main__':
        main()

    



