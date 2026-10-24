import array
import os

import numpy as np
import yaml
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, FloatingPointRange

import cv2
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Polygon, Point32
from cv_bridge import CvBridge

from follow_lane_msg.msg import FollowLaneDebug, LaneMask

EXTRINSICS = 'extrinsics_800x600.yaml'
# Referenzaufloesung, fuer die alle Pixelwerte im Code und die RoI-Parameter gelten.
# Andere Aufloesungen mit gleichem Seitenverhaeltnis werden darauf umgerechnet.
REF_W, REF_H = 800, 600


class detect_lane(Node):
    def __init__(self):
        super().__init__('detect_lane')

        self.get_logger().info('Started detect_lane Node.')

        self.bridge = CvBridge()


        # ---------------------------------
        # Parameter:
        # ---------------------------------

        # --- Wiederverwendbare Ranges ---
        byte = ParameterDescriptor(integer_range=[IntegerRange(from_value=0,    to_value=255,  step=1)])
        # Obergrenze deckt 1920x1080 ab, negativ weil RoI-Ecken ausserhalb des Bildes liegen duerfen
        crop = ParameterDescriptor(integer_range=[IntegerRange(from_value=-500, to_value=2200, step=1)])

        # --- LAB-Parameter (Farbfilter der Linien) ---
        # L = Helligkeit, A = gruen(<128)..rot(>128), B = blau(<128)..gelb(>128); 128 = farblos.
        # Startwerte gemessen auf BEV-Bildern nach CLAHE (docs/bev-tests/README.md, 22.10.):
        #   Gelb       L 170-244  A 115-127  B 152-210   -> erkennbar am hohen B
        #   Weiss      L 210-255  A 120-128  B 118-157   -> erkennbar am hohen L, A/B nahe 128
        #   Matte      L  35-185  A 124-132  B 100-133
        self.declare_parameter('lab_l_white_l', 200, byte)
        self.declare_parameter('lab_l_white_h', 255, byte)
        self.declare_parameter('lab_a_white_l', 112, byte)
        self.declare_parameter('lab_a_white_h', 140, byte)
        self.declare_parameter('lab_b_white_l', 110, byte)
        self.declare_parameter('lab_b_white_h', 145, byte)
        self.declare_parameter('lab_l_yellow_l', 140, byte)
        self.declare_parameter('lab_l_yellow_h', 255, byte)
        self.declare_parameter('lab_a_yellow_l', 100, byte)
        self.declare_parameter('lab_a_yellow_h', 140, byte)
        self.declare_parameter('lab_b_yellow_l', 150, byte)
        self.declare_parameter('lab_b_yellow_h', 255, byte)

        # --- Blau (LAB) ---
        # Gemessen am blauen Klemmbrettrand (23.10.): L 14-108, A 143-176, B 56-99;
        # Rest des Bildes: A 120-133, B 115-164 -> erkennbar an niedrigem B + erhoehtem A.
        self.declare_parameter('lab_l_blue_l', 10,  byte)
        self.declare_parameter('lab_l_blue_h', 255, byte)
        self.declare_parameter('lab_a_blue_l', 135, byte)
        self.declare_parameter('lab_a_blue_h', 200, byte)
        self.declare_parameter('lab_b_blue_l', 0,   byte)
        self.declare_parameter('lab_b_blue_h', 105, byte)

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
        # Gleicht ueber-/unterbelichtete Bildbereiche vor der LAB-Farbfilterung an, damit
        # wechselnde Lichtverhaeltnisse die festen Schwellwerte nicht verfaelschen.
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
        self.sub_BEV = self.create_subscription(Image,'/lokal/bev', self.cbGetBEV,1)


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
        self.img_height = None   # Groesse des Kamerabilds, wird pro Frame in cbFindLane gesetzt
        self.img_width  = None

        self.intersection = False

        self.px_scale = 1.0
        self._metric_ref = (REF_W, REF_H)
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
            # Die warp_matrix gilt fuer Bildpixel dieser Groesse -- bei anderer Kamera-
            # aufloesung (gleiches Seitenverhaeltnis) werden die Eckpunkte mitskaliert.
            self._metric_ref = (int(d.get('image_width', REF_W)), int(d.get('image_height', REF_H)))
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

    def _apply_metric_warp_corners(self, w, h):
        """Setzt die vier top_/bottom_-Punkte auf die Bildecken, die die warp_matrix
        auf die Ecken des 400x400-Ausschnitts abbildet. crop_img() baut daraus per
        getPerspectiveTransform dieselbe Homographie wieder auf, und debug_view kann
        sie unveraendert invertieren -- eine einzige Quelle der Wahrheit.
        w, h: tatsaechliche Bildgroesse; die Punkte werden von der Kalibriergroesse
        darauf skaliert (K skaliert linear mit, die Bodengeometrie bleibt gleich)."""
        size = self._crop_im_size
        out = np.float32([[0, 0], [size, 0], [size, size], [0, size]]).reshape(-1, 1, 2)
        src = cv2.perspectiveTransform(out, np.linalg.inv(self._metric_M)).reshape(-1, 2)
        src = src * np.array([w / self._metric_ref[0], h / self._metric_ref[1]])
        (self.top_left_x, self.top_left_y) = (int(round(src[0][0])), int(round(src[0][1])))
        (self.top_right_x, self.top_right_y) = (int(round(src[1][0])), int(round(src[1][1])))
        (self.bottom_right_x, self.bottom_right_y) = (int(round(src[2][0])), int(round(src[2][1])))
        (self.bottom_left_x, self.bottom_left_y) = (int(round(src[3][0])), int(round(src[3][1])))

    def cbGetRawImage(self,img):
        self.raw_image = img

        self.detectBarrier

        # self.cbFindLane(img)

    def cbGetBEV(self,img):
        self.BEV=img

        self.cbFindLaneBEV(img)

    def detectBarrier(self,img):
        mask_blue = self.blue_mask(img)



    def cbFindLaneBEV(self, img):
        if self.is_running:
            return
        self.is_running = True
        # Alles ab hier im try: sonst wuerde eine Exception vor dem try das Flag
        # dauerhaft auf True lassen (finally liefe nie) und der Node verarbeitet
        # still keine Bilder mehr.
        try:
            # Alle oben deklarierten Parameter als self.<name>-Attribute verfügbar machen
            param_names = [
                'lab_l_white_l', 'lab_l_white_h', 'lab_a_white_l', 'lab_a_white_h',
                'lab_b_white_l', 'lab_b_white_h',
                'lab_l_yellow_l', 'lab_l_yellow_h', 'lab_a_yellow_l', 'lab_a_yellow_h',
                'lab_b_yellow_l', 'lab_b_yellow_h',
                'lab_l_blue_l', 'lab_l_blue_h', 'lab_a_blue_l', 'lab_a_blue_h',
                'lab_b_blue_l', 'lab_b_blue_h',
                'top_left_x', 'top_left_y', 'top_right_x', 'top_right_y',
                'bottom_left_x', 'bottom_left_y', 'bottom_right_x', 'bottom_right_y',
                'detection_row_factor', 'min_lane_width',
                'clahe_clip_limit', 'clahe_tile_grid_size', 'lane_mask_stride',
            ]
            for name in param_names:
                setattr(self, name, self.get_parameter(name).value)

            cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
            self.img_height, self.img_width = cv_image.shape[:2]
            
            res_m= 0.00125
            px_per_cm = 0.01/res_m
            lane_error_scale = 20
            #min_edge_len = 5* pxpercm

            detection_row_factor = self.detection_row_factor

            self.raw_img = cv_image
            mask_yellow, mask_white = self.color_masks(cv_image)

                
            k = max(3, int(round(5 * self.px_scale)) | 1)    # 5x5 bei 800x600, ungerade
            kernel = np.ones((k, k), np.uint8)

            mask_yellow = cv2.erode(mask_yellow,kernel,iterations =1 )

            mask_white = cv2.erode(mask_white,kernel,iterations =1 )


            cropped_white = mask_white
            cropped_yellow = mask_yellow

            white_alternative = int(len(cropped_white[0]) * 0.65)
            yellow_alternative = int(len(cropped_yellow[0]) * 0.35)

            # Gelb fix am konfigurierten detection_row_factor
            yellow_row_factor = self.detection_row_factor
            center_yellow_y = int(len(cropped_yellow)*yellow_row_factor)
            center_yellow_x = self.get_x_for_driving(cropped_yellow, center_yellow_y, yellow_alternative, left_line=True)

            # Weiße Maske: alles links von gelb + min_lane_width ausblenden
            mask_white_filtered = cropped_white.copy()
            corridor_start = max(0, int(center_yellow_x) + self.min_lane_width)
            mask_white_filtered[:, :corridor_start] = 0

            # Weiß sucht weiter unten falls nicht gefunden
            center_white_y = int(len(cropped_white)*detection_row_factor)
            center_white_x = self.get_x_for_driving(mask_white_filtered, center_white_y, white_alternative, left_line=False)

            while center_white_x == white_alternative and detection_row_factor <= 0.95:
                detection_row_factor += 0.05
                center_white_y = int(len(cropped_white)*detection_row_factor)
                center_white_x = self.get_x_for_driving(mask_white_filtered, center_white_y, white_alternative, left_line=False)

            

            if center_white_x <= center_yellow_x:
                if center_white_x > int(len(cropped_white[0]) * 0.4):
                    center_yellow_x = yellow_alternative
                else:
                    center_white_x = white_alternative


            lane_center = (center_white_x + center_yellow_x) / 2
            offset_px = len(cropped_white[0]) / 2 - lane_center
            offset_cm = offset_px / px_per_cm
            lane_error = offset_cm / lane_error_scale

            lane_error = 1-(lane_center / len(cropped_white[0]) * 2)

            polies, horizontal, vertical, corners, intersection_corners = \
                self.detect_intersection(mask_white,mask_yellow)
            self.intersection = len(intersection_corners) > 0

            if self.intersection:
                intersection_distance = self.intersection_distance(intersection_corners)    

            


            msg_error = Float64()
            msg_error.data = lane_error

            self.pub_lane_error.publish(msg_error)
            self.pub_debug_follow_lane(center_yellow_x, center_yellow_y, center_white_x, center_white_y, lane_error, int(lane_center), cv_image, polies, horizontal, vertical,
                                       corners, intersection_corners, bev=True)
            self.publish_mask_debug(cropped_yellow, cropped_white,
                                    center_yellow_x, center_yellow_y, center_white_x, center_white_y)
            # Ungefilterte Masken: die links der gelben Linie ausgeblendeten weissen Pixel
            # braucht nur der PID; fuer die Costmap trennt lane_costmap_node selbst.
            self.publish_lane_mask(mask_yellow, mask_white, img.header.stamp)

        except Exception as e:
            self.get_logger().error(f"cbFindLane failed: {e}")
        finally:
            self.is_running = False

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
                'lab_l_white_l', 'lab_l_white_h', 'lab_a_white_l', 'lab_a_white_h',
                'lab_b_white_l', 'lab_b_white_h',
                'lab_l_yellow_l', 'lab_l_yellow_h', 'lab_a_yellow_l', 'lab_a_yellow_h',
                'lab_b_yellow_l', 'lab_b_yellow_h',
                'lab_l_blue_l', 'lab_l_blue_h', 'lab_a_blue_l', 'lab_a_blue_h',
                'lab_b_blue_l', 'lab_b_blue_h',
                'top_left_x', 'top_left_y', 'top_right_x', 'top_right_y',
                'bottom_left_x', 'bottom_left_y', 'bottom_right_x', 'bottom_right_y',
                'detection_row_factor', 'min_lane_width',
                'clahe_clip_limit', 'clahe_tile_grid_size',
            ]
            for name in param_names:
                setattr(self, name, self.get_parameter(name).value)

            cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
            self.img_height, self.img_width = cv_image.shape[:2]
            # Pixelgroessen im Code (Kernel, Mindestlaengen, RoI-Parameter) sind fuer
            # REF_W x REF_H ausgelegt und werden mit diesem Faktor auf die echte
            # Aufloesung umgerechnet (z.B. 1640x1232 -> 2.05).
            self.px_scale = self.img_width / REF_W

            # Bei aktiver metrischer Vogelperspektive die vier RoI-Punkte aus der
            # warp_matrix ableiten -- ueberschreibt die eben gelesenen Parameterwerte.
            if self.get_parameter('use_metric_warp').value and self._metric_M is not None:
                self._apply_metric_warp_corners(self.img_width, self.img_height)
            elif (self.img_width, self.img_height) != (REF_W, REF_H):
                # Handgetunte RoI-Parameter sind Pixel im REF_W x REF_H-Bild
                for name in ('top_left', 'top_right', 'bottom_left', 'bottom_right'):
                    setattr(self, name + '_x', int(round(getattr(self, name + '_x') * self.img_width / REF_W)))
                    setattr(self, name + '_y', int(round(getattr(self, name + '_y') * self.img_height / REF_H)))

            detection_row_factor = self.detection_row_factor

            self.raw_img = cv_image
            mask_yellow, mask_white = self.color_masks(cv_image)

                
            k = max(3, int(round(5 * self.px_scale)) | 1)    # 5x5 bei 800x600, ungerade
            kernel = np.ones((k, k), np.uint8)

            mask_yellow = cv2.erode(mask_yellow,kernel,iterations =1 )

            mask_white = cv2.erode(mask_white,kernel,iterations =1 )


            cropped_white = self.crop_img(mask_white)
            cropped_yellow = self.crop_img(mask_yellow)

            white_alternative = int(len(cropped_white[0]) * 0.95)
            yellow_alternative = int(len(cropped_yellow[0]) * 0.05)

            # Gelb fix am konfigurierten detection_row_factor
            yellow_row_factor = self.detection_row_factor
            center_yellow_y = int(len(cropped_yellow)*yellow_row_factor)
            center_yellow_x = self.get_x_for_driving(cropped_yellow, center_yellow_y, yellow_alternative, left_line=True)

            # Weiße Maske: alles links von gelb + min_lane_width ausblenden
            mask_white_filtered = cropped_white.copy()
            corridor_start = max(0, int(center_yellow_x) + self.min_lane_width)
            mask_white_filtered[:, :corridor_start] = 0

            # Weiß sucht weiter unten falls nicht gefunden
            center_white_y = int(len(cropped_white)*detection_row_factor)
            center_white_x = self.get_x_for_driving(mask_white_filtered, center_white_y, white_alternative, left_line=False)

            while center_white_x == white_alternative and detection_row_factor <= 0.95:
                detection_row_factor += 0.05
                center_white_y = int(len(cropped_white)*detection_row_factor)
                center_white_x = self.get_x_for_driving(mask_white_filtered, center_white_y, white_alternative, left_line=False)

            

            if center_white_x <= center_yellow_x:
                if center_white_x > int(len(cropped_white[0]) * 0.4):
                    center_yellow_x = yellow_alternative
                else:
                    center_white_x = white_alternative

            lane_center = (center_white_x + center_yellow_x) / 2
            lane_error = 1-(lane_center / len(cropped_white[0]) * 2)

            polies, horizontal, vertical, corners, intersection_corners = \
                self.detect_intersection(mask_white,mask_yellow)
            self.intersection = len(intersection_corners) > 0

            if self.intersection:
                intersection_distance = self.intersection_distance(intersection_corners)    

            


            msg_error = Float64()
            msg_error.data = lane_error

            self.pub_lane_error.publish(msg_error)
            self.pub_debug_follow_lane(center_yellow_x, center_yellow_y, center_white_x, center_white_y, lane_error, int(lane_center), cv_image, polies, horizontal, vertical,
                                       corners, intersection_corners)
            self.publish_mask_debug(cropped_yellow, cropped_white,
                                    center_yellow_x, center_yellow_y, center_white_x, center_white_y)

        except Exception as e:
            self.get_logger().error(f"cbFindLane failed: {e}")
        finally:
            self.is_running = False

    def publish_mask_debug(self, mask_yellow, mask_white,
                           center_yellow_x, center_yellow_y, center_white_x, center_white_y):
        if self.pub_mask_debug.get_subscription_count() == 0:
            return

        overlay = np.zeros((mask_yellow.shape[0], mask_yellow.shape[1], 3), np.uint8)
        both = (mask_yellow > 0) & (mask_white > 0)
        overlay[mask_white > 0]  = (255, 255, 255)
        overlay[mask_yellow > 0] = (0, 255, 255)
        overlay[both] = (0, 0, 255)

        cv2.circle(overlay, (center_yellow_x, center_yellow_y), 6, (0, 255, 0),   -1, cv2.LINE_AA)
        cv2.circle(overlay, (center_white_x,  center_white_y),  6, (255, 0, 255), -1, cv2.LINE_AA)

        success, encoded = cv2.imencode('.jpg', overlay)
        if not success:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        # array('B') statt bytes: rclpy prueft bytes sonst Byte fuer Byte in Python (~0,7 s/MB)
        msg.data = array.array('B', encoded.tobytes())
        self.pub_mask_debug.publish(msg)

    def publish_lane_mask(self, mask_yellow, mask_white, source_stamp):
        """Linienpixel der BEV-Masken fuer lane_costmap_node (/detect/lane_mask).

        Die Koordinaten sind Pixel in /lokal/bev; lane_costmap_node rechnet sie ueber
        Parameter/bev.yaml in Meter um. source_stamp ist der Stempel des Kamerabilds:
        Nav2s obstacle_layer transformiert die Spurwolke per TF auf genau diesen
        Zeitpunkt, ein spaeterer (now()) verschoebe die Waende gegen die Roboterpose.
        """
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
        msg.image_height, msg.image_width = mask_yellow.shape[:2]
        msg.yellow_x = yellow_xs.astype(np.int32).tolist()
        msg.yellow_y = yellow_ys.astype(np.int32).tolist()
        msg.white_x = white_xs.astype(np.int32).tolist()
        msg.white_y = white_ys.astype(np.int32).tolist()
        self.pub_lane_mask.publish(msg)

    def pub_debug_follow_lane(self, center_yellow_x, center_yellow_y, center_white_x, center_white_y, lane_error, lane_center, img, polies, horizontal, vertical,
                              corners=(), intersection_corners=(), bev=False):
        follow_lane_debug_info = FollowLaneDebug()
        follow_lane_debug_info.center_white_x = center_white_x
        follow_lane_debug_info.center_white_y = center_white_y
        follow_lane_debug_info.center_yellow_x = center_yellow_x
        follow_lane_debug_info.center_yellow_y = center_yellow_y
        follow_lane_debug_info.lane_center = lane_center
        follow_lane_debug_info.lane_error = lane_error
        # 0 = das Bild ist schon die BEV (cbFindLaneBEV): debug_view zeichnet die Punkte direkt,
        # ohne RoI-Viereck und ohne Rueckrechnung ueber die alte Perspektive.
        follow_lane_debug_info.crop_im_size = 0 if bev else self._crop_im_size

        success, encoded = cv2.imencode('.jpg', img)
        if success:
            follow_lane_debug_info.raw_image.format = 'jpeg'
            # array('B') statt bytes: rclpy prueft bytes sonst Byte fuer Byte in Python (~0,7 s/MB)
            follow_lane_debug_info.raw_image.data = array.array('B', encoded.tobytes())

        follow_lane_debug_info.top_left_x     = self.top_left_x
        follow_lane_debug_info.top_left_y     = self.top_left_y
        follow_lane_debug_info.top_right_x    = self.top_right_x
        follow_lane_debug_info.top_right_y    = self.top_right_y
        follow_lane_debug_info.bottom_left_x  = self.bottom_left_x
        follow_lane_debug_info.bottom_left_y  = self.bottom_left_y
        follow_lane_debug_info.bottom_right_x = self.bottom_right_x
        follow_lane_debug_info.bottom_right_y = self.bottom_right_y

        for poly in polies or []:
            p = Polygon()
            p.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in poly.reshape(-1, 2)]
            follow_lane_debug_info.polygons.append(p)

        for lines, field in ((horizontal, follow_lane_debug_info.horizontal_lines),
                             (vertical, follow_lane_debug_info.vertical_lines)):
            for p1, p2 in lines or []:
                field.append(Polygon(points=[Point32(x=float(p1[0]), y=float(p1[1]), z=0.0),
                                             Point32(x=float(p2[0]), y=float(p2[1]), z=0.0)]))

        self.pub_debug_image.publish(follow_lane_debug_info)

    def intersection_distance(self,intersection_corners):
        nearest_corner = 0
        for corner in intersection_corners:
            if corner[0][1] > nearest_corner:
                nearest_corner = corner[0][1]
        return nearest_corner


    def get_x_beside(self, mask, y, x_start, direction, no_lane_value, min_gap=5):
        """get_x_for_driving, aber nur auf einer Seite von x_start: direction=-1 sucht
        links davon die naechste Kante, +1 rechts davon. Noetig bei der kombinierten
        Maske, sonst findet die Suche den Streifen der Ecke selbst."""
        x_start = int(x_start)
        if direction < 0:
            x_end = x_start - min_gap
            if x_end <= 0:
                return no_lane_value
            return self.get_x_for_driving(mask[:, :x_end], int(y), no_lane_value, left_line=True)
        x_begin = x_start + min_gap
        if x_begin >= mask.shape[1]:
            return no_lane_value
        x = self.get_x_for_driving(mask[:, x_begin:], int(y), None, left_line=False)
        return no_lane_value if x is None else x + x_begin

    def is_intersection_free(self,intersection_corners,mask):
        """mask = kombinierte Maske (weiss | gelb), Koordinaten wie die Ecken (volles Bild)."""
        pts = np.array([p for p, b in intersection_corners]).reshape(-1, 2)
        pts = pts[np.argsort(pts[:, 1])]
        
        shift_hor = np.array([30, 0])
        shift_ver = np.array([30, 0])  
        if len(pts) == 2:
            if pts[0][0] >= self.img_width/2 and pts[1][0] >= self.img_width/2:
                pt3_x = self.get_x_beside(mask, pts[1][1], pts[1][0], -1, None)
                pt4_x = self.get_x_beside(mask, pts[0][1], pts[0][0], -1, None)
                if pt3_x is not None and pt4_x is not None:
                    straight_lane = np.array([pts[0],pts[1],[pt3_x,pts[1][1]], [pt4_x,pts[0][1]]],np.int32)
                right_lane = [pts[0],pts[1],pts[1]-shift_hor,pts[0]-shift_hor]
            elif pts[0][0] <= self.img_width/2 and pts[1][0] <= self.img_width/2:
                pt3_x = self.get_x_beside(mask, pts[1][1], pts[1][0], +1, None)
                pt4_x = self.get_x_beside(mask, pts[0][1], pts[0][0], +1, None)
                if pt3_x is not None and pt4_x is not None:
                    straight_lane = np.array([pts[0],pts[1],[pt3_x,pts[1][1]], [pt4_x,pts[0][1]]],np.int32)
                left_lane = [pts[0],pts[1],pts[1]+shift_hor,pts[0]+shift_hor]
            else:
                bot_l, bot_r = pts[np.argsort(pts[:, 0])]
                y_top = self.get_y_for_driving(mask, bot_l[0] + 10, bot_r[0] - 10,
                                               min(bot_l[1], bot_r[1]), None)
                if y_top is not None:
                    top_l = np.array([bot_l[0], y_top])
                    top_r = np.array([bot_r[0], y_top])
                    straight_lane = np.array([top_l, top_r, bot_r, bot_l], np.int32)
                    right_lane = [top_r, bot_r, bot_r-shift_hor, top_r-shift_hor]
                    left_lane  = [top_l, bot_l, bot_l+shift_hor, top_l+shift_hor]
        elif len(pts) == 4:
            top_l, top_r = pts[:2][np.argsort(pts[:2, 0])]
            bot_l, bot_r = pts[2:][np.argsort(pts[2:, 0])]
            straight_lane = np.array([top_l, top_r, bot_r, bot_l], np.int32)
            right_lane = [top_r, bot_r, bot_r-shift_hor, top_r-shift_hor]
            left_lane  = [top_l, bot_l, bot_l+shift_hor, top_l+shift_hor]

        left_lane = []
        right_lane = []
        straight_lane =[]

    def detect_corners(self,img):
        corners =cv2.cornerHarris(img,2,3,0.04)
        return corners

    def lines_to_poly(self,img, y_offset):
        polies = []
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE,
                                   offset=(0, y_offset))       
        for contour in contours:
            if cv2.contourArea(contour) < 50 * self.px_scale ** 2:   # Rauschen/Fransen raus
                continue
            polies.append(cv2.approxPolyDP(contour, epsilon=3 * self.px_scale, closed=True))

        return polies

    def get_angle_of_line(self,p1,p2):
        d = p2-p1
        ang = abs(np.degrees(np.arctan2(d[1], d[0])))
        return min(ang, 180 - ang)

    def line_dir(self, p1, p2):
        d = np.asarray(p2, float) - np.asarray(p1, float)
        return np.degrees(np.arctan2(d[1], d[0])) % 180.0

    def angle_diff(self, a, b):
        d = abs(a - b) % 180.0
        return min(d, 180.0 - d)

    def corner_bisector(self, p_prev, p, p_next):
        u1 = (p_prev - p) / np.linalg.norm(p_prev - p)
        u2 = (p_next - p) / np.linalg.norm(p_next - p)
        b = u1 + u2
        n = np.linalg.norm(b)
        return None if n < 1e-6 else b / n

    def _on_border(self, p1, p2, y0):
        return (p1[0] == 0 and p2[0] == 0) or (p1[0] == self.img_width - 1 and p2[0] == self.img_width - 1) or \
               (p1[1] == y0 and p2[1] == y0) or (p1[1] == self.img_height - 1 and p2[1] == self.img_height - 1)
    
    def detect_intersection(self,img_white, img_yellow):
        min_edge_len   = 30 * self.px_scale     # 30 px bei 800x600
        parallel_tol   = 4.0    
        corner_tol     = 10.0   
        min_dot        = 0.7    

        full_img = cv2.bitwise_or(img_white, img_yellow)
        y0 = int(self.img_height * 0.30)
        roi = full_img[int( y0):,:]
        polies = self.lines_to_poly(roi,y0)

        vertical = []
        horizontal = []
        corners_per_poly = []      # pro Fahrbahnlinie: [(eckpunkt, halbierende), ...]
        for poly in polies:
            pts = poly.reshape(-1,2).astype(float)
            len_pts = len(pts)
            if len_pts < 4:
                continue

            # Kanten sammeln (ohne Rand-Kanten und ohne kurze Fransen)
            edge_dirs = []
            for i in range(len_pts):
                p1, p2 = pts[i], pts[(i+1)%len_pts]
                if self._on_border(p1, p2, y0):
                    continue
                if np.linalg.norm(p2-p1) > min_edge_len:
                    edge_dirs.append(self.line_dir(p1, p2))
                    angle = self.get_angle_of_line(p1, p2)
                    if angle < 10:
                        horizontal.append((p1, p2))
                    elif angle >= 80:
                        vertical.append((p1, p2))

            has_parallel = any(self.angle_diff(edge_dirs[a], edge_dirs[b]) <= parallel_tol
                               for a in range(len(edge_dirs)) for b in range(a+1, len(edge_dirs)))
            if not has_parallel:
                continue

            corners = []
            for i in range(len_pts):
                p_prev, p, p_next = pts[i-1], pts[i], pts[(i+1)%len_pts]
                if min(np.linalg.norm(p_prev-p), np.linalg.norm(p_next-p)) < min_edge_len:
                    continue
                if self._on_border(p_prev, p, y0) or \
                   self._on_border(p, p_next, y0):
                    continue
                if abs(self.angle_diff(self.line_dir(p_prev, p), self.line_dir(p, p_next)) - 90) > corner_tol:
                    continue
                b = self.corner_bisector(p_prev, p, p_next)
                if b is not None:
                    corners.append((p, b))
            if corners:
                corners_per_poly.append((poly, corners))

        # Kreuzungsecke = Innen- und Aussenkante DESSELBEN Streifens knicken parallel ab,
        # d.h. zwei Ecken im selben Polygon mit gleich gerichteter Halbierender.
        # Pro Paar wird die konvexe Ecke behalten (Halbierende zeigt ins Polygon) --
        # das ist die Ecke an der Fahrbahnseite (PT1/P2 in der Skizze).
        intersection_corners = []
        for poly, corners in corners_per_poly:
            used = set()
            for i in range(len(corners)):
                for j in range(i+1, len(corners)):
                    if i in used or j in used:
                        continue
                    (c1, b1), (c2, b2) = corners[i], corners[j]
                    if np.dot(b1, b2) <= min_dot:
                        continue
                    used.update((i, j))
                    for c, b in ((c1, b1), (c2, b2)):
                        probe = c + 5.0 * b
                        if cv2.pointPolygonTest(poly, (float(probe[0]), float(probe[1])), False) > 0:
                            intersection_corners.append((c, b))
                            break

        corners = [c for _, cs in corners_per_poly for c in cs]
        return polies, horizontal, vertical, corners, intersection_corners

    def color_masks(self, img):
        """Gelb- und Weiss-Maske aus dem BGR-Bild: CLAHE auf L, dann Schwellen in LAB."""
        lab = self.apply_clahe(img, return_lab=True)
        mask_yellow = cv2.inRange(lab,
            (self.lab_l_yellow_l, self.lab_a_yellow_l, self.lab_b_yellow_l),
            (self.lab_l_yellow_h, self.lab_a_yellow_h, self.lab_b_yellow_h))
        mask_white = cv2.inRange(lab,
            (self.lab_l_white_l, self.lab_a_white_l, self.lab_b_white_l),
            (self.lab_l_white_h, self.lab_a_white_h, self.lab_b_white_h))
        return mask_yellow, mask_white

    def blue_mask(self,img):
        """Blau-Maske aus dem BGR-Bild: CLAHE auf L, dann Schwellen in LAB (Parameter lab_*_blue_*)."""
        lab = self.apply_clahe(img, return_lab=True)
        mask_blue= cv2.inRange(lab,
            (self.lab_l_blue_l, self.lab_a_blue_l, self.lab_b_blue_l),
            (self.lab_l_blue_h, self.lab_a_blue_h, self.lab_b_blue_h))
        return mask_blue


    def apply_clahe(self, img, return_lab=False):
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l_channel, a_channel, b_channel = cv2.split(lab)

        tile_size = max(1, int(self.clahe_tile_grid_size))
        clahe = cv2.createCLAHE(clipLimit=float(self.clahe_clip_limit),
                                 tileGridSize=(tile_size, tile_size))
        l_channel = clahe.apply(l_channel)

        lab = cv2.merge((l_channel, a_channel, b_channel))
        if return_lab:
            return lab
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

    def get_y_for_driving(self, mask, x_from, x_to, y_start, no_lane_value, min_gap=5):
        """Gegenstueck zu get_x_for_driving, aber spaltenweise: sucht in den Spalten
        x_from..x_to von y_start aus NACH OBEN die naechste waagerechte Kante
        (z.B. die querliegende Linie am Ende einer ┴-Kreuzung).
        min_gap: Zeilen direkt ueber y_start ignorieren, damit nicht die Kante
        an der Ecke selbst gefunden wird."""
        grad = cv2.Sobel(mask, cv2.CV_16S, 0, 1, ksize=3, scale=1, delta=0, borderType=cv2.BORDER_DEFAULT)
        abs_grad = cv2.convertScaleAbs(grad)

        _,th1 = cv2.threshold(abs_grad,127,255,cv2.THRESH_BINARY)

        y_end = int(y_start) - min_gap
        if y_end <= 0:
            return no_lane_value

        a = []
        for col in range(max(0, int(x_from)), min(mask.shape[1], int(x_to))):
            hits = np.where(th1[:y_end, col] == 255)[0]
            if hits.size == 0:
                continue
            a.append(hits[-1])      # groesstes y oberhalb = naechste Kante ueber der Ecke

        if len(a) > 10:
            return int(np.median(a))
        else:
            return no_lane_value

    
    #NOCHMAL Überarbeiten
def main(args=None):    
    # SIGINT selbst behandeln (KeyboardInterrupt), damit der Context beim Aufraeumen noch
    # gilt -- sonst bei Strg+C ExternalShutdownException-/"context is invalid"-Tracebacks.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = detect_lane()
    try:
        # spin_once mit Timeout statt spin(): spin() wartet in C auf die naechste Nachricht,
        # Strg+C kaeme ohne Daten nie an (Knoten haengt beim Beenden).
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:

        node.destroy_node()
        # rclpy raeumt den Context im eigenen SIGINT-Handler ab -- sonst wirft der zweite Aufruf
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

    



