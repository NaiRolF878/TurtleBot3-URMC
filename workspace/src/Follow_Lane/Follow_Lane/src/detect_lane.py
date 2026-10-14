import os

import numpy as np
import yaml
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, FloatingPointRange

import cv2
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

from follow_lane_msg.msg import FollowLaneDebug, LaneMask

EXTRINSICS = 'extrinsics_800x600.yaml'


class detect_lane(Node):
    def __init__(self):
        super().__init__('detect_lane')

        self.get_logger().info('Started detect_lane Node.')

        self.bridge = CvBridge()


        # ---------------------------------
        # Parameter:
        # ---------------------------------

        # --- Wiederverwendbare Ranges ---
        hsv  = ParameterDescriptor(integer_range=[IntegerRange(from_value=0,    to_value=255,  step=1)])
        # Obergrenze deckt 1920x1080 ab, negativ weil RoI-Ecken ausserhalb des Bildes liegen duerfen
        crop = ParameterDescriptor(integer_range=[IntegerRange(from_value=-500, to_value=2200, step=1)])

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
        # use_metric_warp=True nutzt stattdessen die vermessene Bodenebenen-Homographie
        # aus Parameter/camera_info/extrinsics_800x600.yaml (warp_matrix). Die vier
        # top_/bottom_-Punkte werden dann daraus abgeleitet, damit debug_view sie
        # unveraendert zurueckrechnen kann. Default False: erst einschalten, wenn
        # Intrinsik UND Extrinsik neu und sauber vermessen sind (siehe docs/MEMORY.md).
        self.declare_parameter('use_metric_warp', False)
        self.declare_parameter('top_left_x',     159, crop)
        self.declare_parameter('top_left_y',     218, crop)
        self.declare_parameter('top_right_x',    441, crop)
        self.declare_parameter('top_right_y',    218, crop)
        self.declare_parameter('bottom_left_x',  -29, crop)
        self.declare_parameter('bottom_left_y',  382, crop)
        self.declare_parameter('bottom_right_x', 606, crop)
        self.declare_parameter('bottom_right_y', 382, crop)

        # --- Sonstige ---
        # step=0.0: rclpy prueft sonst (wert-from) % step == 0, was mit Fliesskomma zufaellig fehlschlaegt
        self.declare_parameter('detection_row_factor', 0.75,
            ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=0.5, to_value=0.95, step=0.0)]))
        self.declare_parameter('min_lane_width', 80,
            ParameterDescriptor(integer_range=[IntegerRange(from_value=0, to_value=300, step=1)]))

        # --- CLAHE-Parameter ---
        # Gleicht ueber-/unterbelichtete Bildbereiche vor der HSV-Filterung an, damit
        # wechselnde Lichtverhaeltnisse die festen HSV-Schwellwerte nicht verfaelschen.
        self.declare_parameter('clahe_clip_limit', 2.0,
            ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=0.1, to_value=40.0, step=0.0)]))
        self.declare_parameter('clahe_tile_grid_size', 8,
            ParameterDescriptor(integer_range=[IntegerRange(from_value=1, to_value=64, step=1)]))

        # Rasterweite fuers Ausduennen der auf /detect/lane_mask gesendeten Punkte.
        self.declare_parameter('lane_mask_stride', 2,
            ParameterDescriptor(integer_range=[IntegerRange(from_value=1, to_value=20, step=1)]))



        #--- Subscriber ---
        # Tiefe 1, NICHT 10: die Kamera liefert 30 Bilder/s, die Verarbeitung hier ist
        # langsamer. Bei Tiefe 10 stapeln sich bis zu 10 Frames in der Queue und der
        # Regler bekommt Bilder, die bis zu ~330 ms alt sind -- er lenkt dann nach einer
        # Situation, die es nicht mehr gibt. Fuer einen Live-Regelkreis ist das neueste
        # Bild immer besser als ein vollstaendiger, aber veralteter Verlauf.
        self.sub_raw_image = self.create_subscription(Image, '/lokal/raw_image', self.cbGetRawImage ,1)

        #--- Publisher ---
        self.pub_lane_error = self.create_publisher(Float64, '/detect/lane',1)
        self.pub_debug_image = self.create_publisher(FollowLaneDebug, '/detect/lane_debug', 1)
        # Muss auf /compressed enden: image_transport deutet das letzte Namenssegment als Transport
        self.pub_mask_debug = self.create_publisher(CompressedImage, '/detect/mask_debug/compressed', 1)
        # Verlustfreie Pixelkoordinaten der Linienpunkte, z.B. fuer lane_costmap_node --
        # bewusst getrennt vom komprimierten Debug-Bild oben, das nur zum Anschauen gedacht ist.
        self.pub_lane_mask = self.create_publisher(LaneMask, '/detect/lane_mask', 1)

        #Andere Variablen:

        self.min_lane_width = None  #Anpassen

        self.is_running = False

        self._crop_im_size = 400
        self.raw_image = None

        self._metric_M = self._load_metric_warp()

    def _load_metric_warp(self):
        """Bodenebenen-Homographie (src->dst, entzerrtes Bild) aus der Extrinsik-YAML.
        Gibt None zurueck, wenn die Datei fehlt oder nicht zum crop_im_size passt --
        dann bleibt es bei den handgetunten RoI-Punkten."""
        try:
            path = os.path.join(get_package_share_directory('Follow_Lane'),
                                'Parameter', 'camera_info', EXTRINSICS)
            with open(path) as f:
                d = yaml.safe_load(f)
            M = np.array(d['warp_matrix'], dtype=np.float64).reshape(3, 3)
            size = int(d.get('output_size_px', self._crop_im_size))
            if size != self._crop_im_size:
                self.get_logger().warn(
                    f'{EXTRINSICS}: output_size_px={size} != crop_im_size={self._crop_im_size} '
                    '-- metrische Vogelperspektive deaktiviert.')
                return None
            self.get_logger().info(f'Metrische Vogelperspektive aus {EXTRINSICS} geladen '
                                   '(nutzbar per Parameter use_metric_warp).')
            return M
        except Exception as e:
            self.get_logger().warn(f'Keine metrische Kalibrierung ({e}) -- RoI-Punkte werden genutzt.')
            return None

    def _apply_metric_warp_corners(self):
        """Setzt die vier top_/bottom_-Punkte auf die Bildecken, die die warp_matrix
        auf die Ecken des 400x400-Ausschnitts abbildet. crop_img() baut daraus per
        getPerspectiveTransform dieselbe Homographie wieder auf, und debug_view kann
        sie unveraendert invertieren -- eine einzige Quelle der Wahrheit."""
        size = self._crop_im_size
        out = np.float32([[0, 0], [size, 0], [size, size], [0, size]]).reshape(-1, 1, 2)
        src = cv2.perspectiveTransform(out, np.linalg.inv(self._metric_M)).reshape(-1, 2)
        (self.top_left_x, self.top_left_y) = (int(round(src[0][0])), int(round(src[0][1])))
        (self.top_right_x, self.top_right_y) = (int(round(src[1][0])), int(round(src[1][1])))
        (self.bottom_right_x, self.bottom_right_y) = (int(round(src[2][0])), int(round(src[2][1])))
        (self.bottom_left_x, self.bottom_left_y) = (int(round(src[3][0])), int(round(src[3][1])))

    def cbGetRawImage(self,img):
        self.raw_image = img

        self.cbFindLane(img)


    def cbFindLane(self, img):
        if self.is_running:
            return
        self.is_running = True
        # Alles ab hier im try: sonst wuerde eine Exception vor dem try das Flag
        # dauerhaft auf True lassen (finally liefe nie) und der Node verarbeitet
        # still keine Bilder mehr.
        try:
            # Alle oben deklarierten Parameter als self.<name>-Attribute verfügbar machen
            param_names = [
                'hue_white_l', 'hue_white_h', 'saturation_white_l', 'saturation_white_h',
                'lightness_white_l', 'lightness_white_h',
                'hue_yellow_l', 'hue_yellow_h', 'saturation_yellow_l', 'saturation_yellow_h',
                'lightness_yellow_l', 'lightness_yellow_h',
                'top_left_x', 'top_left_y', 'top_right_x', 'top_right_y',
                'bottom_left_x', 'bottom_left_y', 'bottom_right_x', 'bottom_right_y',
                'detection_row_factor', 'min_lane_width',
                'clahe_clip_limit', 'clahe_tile_grid_size', 'lane_mask_stride',
            ]
            for name in param_names:
                setattr(self, name, self.get_parameter(name).value)

            # Bei aktiver metrischer Vogelperspektive die vier RoI-Punkte aus der
            # warp_matrix ableiten -- ueberschreibt die eben gelesenen Parameterwerte.
            if self.get_parameter('use_metric_warp').value and self._metric_M is not None:
                self._apply_metric_warp_corners()

            detection_row_factor = self.detection_row_factor

            # Stempel der Quelle sichern, bevor img unten ueberschrieben wird: Nav2s
            # obstacle_layer transformiert die Spurwolke per TF auf genau diesen Zeitpunkt.
            # Ein spaeterer Stempel (now()) wuerde die Waende gegenueber der Roboterpose
            # verschieben, zu der das Bild tatsaechlich aufgenommen wurde.
            source_stamp = img.header.stamp

            cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
            self.raw_img = cv_image
            img = self.apply_clahe(cv_image)
            img = self.crop_img(img)

            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            mask_yellow = cv2.inRange(hsv,
                               (self.hue_yellow_l,self.saturation_yellow_l, self.lightness_yellow_l),
                               (self.hue_yellow_h,self.saturation_yellow_h, self.lightness_yellow_h),)

            mask_white = cv2.inRange(hsv,
                               (self.hue_white_l,self.saturation_white_l, self.lightness_white_l),
                               (self.hue_white_h,self.saturation_white_h, self.lightness_white_h),)

                
            kernel = np.ones((5, 5), np.uint8)    

            mask_yellow = cv2.erode(mask_yellow,kernel,iterations =1 )


            kernel = np.ones((5, 5), np.uint8)    

            mask_white = cv2.erode(mask_white,kernel,iterations =1 )

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
            lane_error = 1-(lane_center / len(img[0]) * 2)

            msg_error = Float64()
            msg_error.data = lane_error

            self.pub_lane_error.publish(msg_error)
            self.pub_debug_follow_lane(center_yellow_x, center_yellow_y, center_white_x, center_white_y, lane_error, int(lane_center), cv_image)
            self.publish_mask_debug(mask_yellow, mask_white,
                                    center_yellow_x, center_yellow_y, center_white_x, center_white_y)
            # Gefilterte Weissmaske: die links der gelben Linie ausgeblendeten Pixel
            # haelt der Detektor selbst fuer unzuverlaessig -- als virtuelle Waende in
            # der Costmap wuerden sie den freien Korridor blockieren.
            self.publish_lane_mask(mask_yellow, mask_white_filtered, source_stamp)

        except Exception as e:
            self.get_logger().error(f"cbFindLane failed: {e}")
        finally:
            self.is_running = False

    def publish_mask_debug(self, mask_yellow, mask_white,
                           center_yellow_x, center_yellow_y, center_white_x, center_white_y):
        # Nur encodieren, wenn jemand zuschaut -- das JPEG kostet sonst pro Frame Rechenzeit
        if self.pub_mask_debug.get_subscription_count() == 0:
            return

        overlay = np.zeros((mask_yellow.shape[0], mask_yellow.shape[1], 3), np.uint8)
        both = (mask_yellow > 0) & (mask_white > 0)
        overlay[mask_white > 0]  = (255, 255, 255)
        overlay[mask_yellow > 0] = (0, 255, 255)
        # Rot = beide Masken treffen dasselbe Pixel, also ueberlappende HSV-Bereiche
        overlay[both] = (0, 0, 255)

        # Gruen/Magenta, weil Gelb, Weiss und Rot hier schon belegt sind.
        # Keine Ruecktransformation noetig: die center_* liegen bereits im entzerrten Bild.
        cv2.circle(overlay, (center_yellow_x, center_yellow_y), 6, (0, 255, 0),   -1, cv2.LINE_AA)
        cv2.circle(overlay, (center_white_x,  center_white_y),  6, (255, 0, 255), -1, cv2.LINE_AA)

        success, encoded = cv2.imencode('.jpg', overlay)
        if not success:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()
        self.pub_mask_debug.publish(msg)

    def publish_lane_mask(self, mask_yellow, mask_white, source_stamp):
        # Nur berechnen, wenn jemand zuschaut -- gleiche Begruendung wie bei publish_mask_debug.
        if self.pub_lane_mask.get_subscription_count() == 0:
            return

        # Schon hier ausduennen: lane_costmap_node verwirft ohnehin die meisten Punkte
        # wieder, in voller Aufloesung waeren es zehntausende int32 pro Frame.
        stride = max(1, int(self.lane_mask_stride))
        yellow_ys, yellow_xs = np.where(mask_yellow[::stride, ::stride] > 0)
        white_ys, white_xs = np.where(mask_white[::stride, ::stride] > 0)
        yellow_ys, yellow_xs = yellow_ys * stride, yellow_xs * stride
        white_ys, white_xs = white_ys * stride, white_xs * stride

        msg = LaneMask()
        msg.header.stamp = source_stamp
        msg.crop_im_size = self._crop_im_size
        msg.yellow_x = yellow_xs.astype(np.int32).tolist()
        msg.yellow_y = yellow_ys.astype(np.int32).tolist()
        msg.white_x = white_xs.astype(np.int32).tolist()
        msg.white_y = white_ys.astype(np.int32).tolist()
        self.pub_lane_mask.publish(msg)

    def pub_debug_follow_lane(self, center_yellow_x, center_yellow_y, center_white_x, center_white_y, lane_error, lane_center, img):
        follow_lane_debug_info = FollowLaneDebug()
        follow_lane_debug_info.center_white_x = center_white_x
        follow_lane_debug_info.center_white_y = center_white_y
        follow_lane_debug_info.center_yellow_x = center_yellow_x
        follow_lane_debug_info.center_yellow_y = center_yellow_y
        follow_lane_debug_info.lane_center = lane_center
        follow_lane_debug_info.lane_error = lane_error
        follow_lane_debug_info.crop_im_size = self._crop_im_size

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

    def apply_clahe(self, img):
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l_channel, a_channel, b_channel = cv2.split(lab)

        tile_size = max(1, int(self.clahe_tile_grid_size))
        clahe = cv2.createCLAHE(clipLimit=float(self.clahe_clip_limit),
                                 tileGridSize=(tile_size, tile_size))
        l_channel = clahe.apply(l_channel)

        lab = cv2.merge((l_channel, a_channel, b_channel))
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

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
        # Betrag, sonst enthaelt th1 nur linke Kanten (rechte sind negativ) und [-1]/[0] liefern dasselbe
        abs_grad = cv2.convertScaleAbs(grad)

        _,th1 = cv2.threshold(abs_grad,127,255,cv2.THRESH_BINARY)

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
            return int(np.median(a))
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

        node.destroy_node()
        # rclpy raeumt den Context im eigenen SIGINT-Handler ab -- sonst wirft der zweite Aufruf
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

    



