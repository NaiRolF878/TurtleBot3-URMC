"""Palette der Storage-Aufgabe im Kamerabild, gemessen gegen den Magneten.

Ein:  /lokal/raw_image (entzerrt), /control/storage_state
Aus:  /detect/pallet (PalletDetection, jedes verarbeitete Bild, auch ohne Fund),
      /detect/pallet_debug/compressed (nur mit Zuschauer)
"""
import array

import cv2
import numpy as np

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from follow_lane_msg.msg import PalletDetection
from Follow_Lane.src.common import (
    StorageState, frange, irange, read_params, require_params, run_node, storage_state_from_msg)

CLAHE_CLIP_LIMIT = 2.0
CLAHE_TILE_GRID_SIZE = 8
# Gesucht wird ab diesem Anteil der Bildhoehe bis zum Magneten.
ROI_TOP = 0.3
# Form: Flaeche (px), Breite/Hoehe mindestens, Breite hoechstens (Anteil der Bildbreite).
MIN_AREA_PX = 200
MIN_ASPECT = 2.0
MAX_WIDTH = 0.7
# Hier ist keine Palette im Bild oder sie wird nicht gebraucht.
IDLE_STATES = (StorageState.IDLE, StorageState.ERKUNDEN, StorageState.ZUR_ZONE_DREHEN,
               StorageState.ZURUECKDREHEN, StorageState.FERTIG, StorageState.FEHLER)


class pallet_node(Node):
    """Die Kamera sieht den Magneten: seine Lage im Bild steht in magnet_x und
    magnet_top (Anteil der Bildbreite bzw. -hoehe, vor Ort im Debugbild ablesen).
    Gesucht wird nur oberhalb des Magneten ab ROI_TOP.

    Die Palette hat keine Markierung. Erkannt wird sie an ihrer Farbe (LAB-Bereich
    lab_*_pallet_*) und Form: flach und breit, nicht bis an den Bildrand (lange Linien)
    und nicht breiter als MAX_WIDTH. Von mehreren gilt die unterste, also die naechste.
    """

    def __init__(self):
        super().__init__('pallet_node')

        # --- Parameter ---
        byte = irange(0, 255)
        self._param_names = [
            'lab_l_pallet_l', 'lab_l_pallet_h', 'lab_a_pallet_l', 'lab_a_pallet_h',
            'lab_b_pallet_l', 'lab_b_pallet_h', 'magnet_x', 'magnet_top',
        ]
        self.declare_parameter('lab_l_pallet_l', Parameter.Type.INTEGER, byte)
        self.declare_parameter('lab_l_pallet_h', Parameter.Type.INTEGER, byte)
        self.declare_parameter('lab_a_pallet_l', Parameter.Type.INTEGER, byte)
        self.declare_parameter('lab_a_pallet_h', Parameter.Type.INTEGER, byte)
        self.declare_parameter('lab_b_pallet_l', Parameter.Type.INTEGER, byte)
        self.declare_parameter('lab_b_pallet_h', Parameter.Type.INTEGER, byte)
        self.declare_parameter('magnet_x', Parameter.Type.DOUBLE, frange(0.0, 1.0))
        self.declare_parameter('magnet_top', Parameter.Type.DOUBLE, frange(0.0, 1.0))
        require_params(self, self._param_names)
        read_params(self, self._param_names)

        # --- Subscriber ---
        self.sub_state = self.create_subscription(
            String, '/control/storage_state', self.cb_state, 1)
        self.sub_image = self.create_subscription(Image, '/lokal/raw_image', self.cb_image, 1)

        # --- Publisher ---
        self.pub_pallet = self.create_publisher(PalletDetection, '/detect/pallet', 1)
        self.pub_debug = self.create_publisher(CompressedImage, '/detect/pallet_debug/compressed', 1)

        # --- Zustand ---
        self.bridge = CvBridge()
        self.state = StorageState.IDLE

        self.get_logger().info('Started pallet_node.')

    # --- Callbacks ---

    def cb_state(self, msg):
        self.state = storage_state_from_msg(msg)

    def cb_image(self, img):
        # Mit Zuschauer auch im Leerlauf, damit sich magnet_* und die Farben einstellen lassen.
        if self.state in IDLE_STATES and self.pub_debug.get_subscription_count() == 0:
            return
        read_params(self, self._param_names)
        try:
            self.find_pallet(img)
        except Exception as e:  # noqa: BLE001 -- ein kaputtes Bild darf den Node nicht stoppen
            self.get_logger().error(f'Palettenerkennung fehlgeschlagen: {e}')

    # --- Hilfsmethoden ---

    def find_pallet(self, img):
        image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        h, w = image.shape[:2]
        magnet_u = self.magnet_x * w
        magnet_v = int(self.magnet_top * h)
        top = min(int(ROI_TOP * h), magnet_v)

        l_channel, a_channel, b_channel = cv2.split(cv2.cvtColor(image, cv2.COLOR_BGR2LAB))
        clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP_LIMIT,
                                tileGridSize=(CLAHE_TILE_GRID_SIZE, CLAHE_TILE_GRID_SIZE))
        lab = cv2.merge((clahe.apply(l_channel), a_channel, b_channel))
        mask = cv2.inRange(lab,
            (self.lab_l_pallet_l, self.lab_a_pallet_l, self.lab_b_pallet_l),
            (self.lab_l_pallet_h, self.lab_a_pallet_h, self.lab_b_pallet_h))
        mask[:top] = 0
        mask[magnet_v:] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

        best = None
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) < MIN_AREA_PX:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            if bw < MIN_ASPECT * bh or bw > MAX_WIDTH * w:
                continue
            if x <= 0 or x + bw >= w:
                continue
            if best is None or y + bh > best[1] + best[3]:
                best = (x, y, bw, bh)

        msg = PalletDetection()
        msg.header = img.header
        if best is not None:
            x, y, bw, bh = best
            msg.found = True
            msg.lateral = float((magnet_u - (x + bw / 2.0)) / (w / 2.0))
            msg.gap_px = float(magnet_v - (y + bh))
            msg.width_px = float(bw)
        self.pub_pallet.publish(msg)
        self.publish_debug(image, mask, best, magnet_u, magnet_v, top, msg)

    def publish_debug(self, image, mask, best, magnet_u, magnet_v, top, msg):
        if self.pub_debug.get_subscription_count() == 0:
            return
        view = image.copy()
        view[mask > 0] = (0.5 * view[mask > 0] + (0, 0, 127)).astype(np.uint8)
        w = view.shape[1]
        cv2.line(view, (0, top), (w - 1, top), (255, 200, 0), 1)
        cv2.line(view, (0, magnet_v), (w - 1, magnet_v), (0, 255, 255), 2)
        cv2.line(view, (int(magnet_u), magnet_v), (int(magnet_u), view.shape[0] - 1),
                 (0, 255, 255), 2)
        if best is not None:
            x, y, bw, bh = best
            cv2.rectangle(view, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            text = f'seitlich {msg.lateral:+.2f}  Luecke {msg.gap_px:.0f} px'
        else:
            text = 'keine Palette'
        cv2.putText(view, f'{self.state.name}: {text}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 0), 2, cv2.LINE_AA)
        ok, encoded = cv2.imencode('.jpg', view)
        if not ok:
            return
        out = CompressedImage()
        out.header.stamp = self.get_clock().now().to_msg()
        out.format = 'jpeg'
        out.data = array.array('B', encoded.tobytes())
        self.pub_debug.publish(out)

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down pallet_node.')


def main(args=None):
    run_node(pallet_node, args)


if __name__ == '__main__':
    main()
