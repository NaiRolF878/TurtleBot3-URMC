"""Zonenschilder der Storage-Aufgabe: blauer Kreis mit weisser Ziffer 1-3, per Vorlagen.

Ein:  /lokal/raw_image (entzerrt), /control/storage_state
Aus:  /detect/storage_sign (StorageSign, source "vorlage"), nur im Zustand SCHILD_LESEN,
      /detect/storage_sign_debug/compressed (nur mit Zuschauer)

Vorlagen: TEMPLATE_DIR/digit_<n>.png. Fehlt eine, wird die Ziffer mit einer
OpenCV-Schrift gezeichnet. Vor Ort aufnehmen: Bot vor das Schild stellen, Zustand
SCHILD_LESEN (oder Parameter always_on), dann
    ros2 param set /storage_sign_node capture_digit 2
speichert die Ziffer des groessten Schilds als Vorlage fuer "2".
"""
import array
import os

import cv2
import numpy as np

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from follow_lane_msg.msg import StorageSign
from Follow_Lane.src.common import (
    StorageState, irange, read_params, require_params, run_node, storage_state_from_msg)

DIGITS = (1, 2, 3)
TEMPLATE_DIR = '~/.ros/urmc_storage_templates'
CLAHE_CLIP_LIMIT = 2.0
CLAHE_TILE_GRID_SIZE = 8
# Weiss innen: A und B hoechstens so weit von 128 (farblos).
WHITE_AB_TOL = 20
# Schildform: Flaeche (px), Achsenverhaeltnis der Ellipse, Fuellgrad, weisser Anteil.
MIN_AREA_PX = 150
MIN_AXIS_RATIO = 0.5
MIN_FILL_RATIO = 0.75
MIN_WHITE_RATIO = 0.05
# Vorlagenvergleich: beste Korrelation und Abstand zur zweitbesten.
MIN_SCORE = 0.5
MIN_MARGIN = 0.1
# Normierte Groesse einer Ziffer (Breite, Hoehe) fuer den Vergleich.
TEMPLATE_SIZE = (24, 36)


def normalize_digit(mask):
    """Weisse Ziffer (Maske) auf ihren Rahmen zuschneiden und mit Rand auf
    TEMPLATE_SIZE bringen, Seitenverhaeltnis bleibt. None ohne weisse Pixel."""
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        return None
    crop = mask[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
    tw, th = TEMPLATE_SIZE
    scale = min(tw / crop.shape[1], th / crop.shape[0])
    w, h = max(1, int(round(crop.shape[1] * scale))), max(1, int(round(crop.shape[0] * scale)))
    resized = cv2.resize(crop, (w, h), interpolation=cv2.INTER_AREA)
    out = np.zeros((th, tw), np.uint8)
    x0, y0 = (tw - w) // 2, (th - h) // 2
    out[y0:y0 + h, x0:x0 + w] = resized
    return out


def synthetic_digit(digit):
    canvas = np.zeros((120, 90), np.uint8)
    cv2.putText(canvas, str(digit), (10, 105), cv2.FONT_HERSHEY_DUPLEX, 4.0, 255, 12, cv2.LINE_AA)
    return normalize_digit(canvas)


class storage_sign_node(Node):
    """Blau-Maske (LAB nach CLAHE), dann je Flaeche: rund genug (Ellipse passt; die
    Schranke ist ein Balken und faellt weg) und genug weiss innen. Die weisse Ziffer wird
    normiert und mit jeder Vorlage korreliert; die beste zaehlt, wenn sie eindeutig ist.
    """

    def __init__(self):
        super().__init__('storage_sign_node')

        # --- Parameter ---
        byte = irange(0, 255)
        self._param_names = [
            'enabled', 'always_on', 'lab_l_blue_l', 'lab_l_blue_h', 'lab_a_blue_l',
            'lab_a_blue_h', 'lab_b_blue_l', 'lab_b_blue_h', 'white_l_min', 'capture_digit',
        ]
        self.declare_parameter('enabled', Parameter.Type.BOOL)
        self.declare_parameter('always_on', Parameter.Type.BOOL)
        self.declare_parameter('lab_l_blue_l', Parameter.Type.INTEGER, byte)
        self.declare_parameter('lab_l_blue_h', Parameter.Type.INTEGER, byte)
        self.declare_parameter('lab_a_blue_l', Parameter.Type.INTEGER, byte)
        self.declare_parameter('lab_a_blue_h', Parameter.Type.INTEGER, byte)
        self.declare_parameter('lab_b_blue_l', Parameter.Type.INTEGER, byte)
        self.declare_parameter('lab_b_blue_h', Parameter.Type.INTEGER, byte)
        self.declare_parameter('white_l_min', Parameter.Type.INTEGER, byte)
        self.declare_parameter('capture_digit', Parameter.Type.INTEGER, irange(0, 3))
        require_params(self, self._param_names)
        read_params(self, self._param_names)

        # --- Subscriber ---
        self.sub_state = self.create_subscription(
            String, '/control/storage_state', self.cb_state, 1)
        self.sub_image = self.create_subscription(Image, '/lokal/raw_image', self.cb_image, 1)

        # --- Publisher ---
        self.pub_sign = self.create_publisher(StorageSign, '/detect/storage_sign', 10)
        self.pub_debug = self.create_publisher(
            CompressedImage, '/detect/storage_sign_debug/compressed', 1)

        # --- Zustand ---
        self.bridge = CvBridge()
        self.state = StorageState.IDLE
        self.templates = self.load_templates()

        self.get_logger().info('Started storage_sign_node.')

    # --- Callbacks ---

    def cb_state(self, msg):
        self.state = storage_state_from_msg(msg)

    def cb_image(self, img):
        read_params(self, self._param_names)
        capture = int(self.capture_digit)
        if not self.enabled:
            return
        if self.state != StorageState.SCHILD_LESEN and not self.always_on and not capture:
            return
        try:
            self.find_sign(img, capture)
        except Exception as e:  # noqa: BLE001 -- ein kaputtes Bild darf den Node nicht stoppen
            self.get_logger().error(f'Schilderkennung fehlgeschlagen: {e}')

    # --- Hilfsmethoden ---

    def find_sign(self, img, capture):
        image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        lab = self.lab_clahe(image)
        blue = cv2.inRange(lab,
            (self.lab_l_blue_l, self.lab_a_blue_l, self.lab_b_blue_l),
            (self.lab_l_blue_h, self.lab_a_blue_h, self.lab_b_blue_h))
        blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        white = cv2.inRange(lab, (int(self.white_l_min), 128 - WHITE_AB_TOL, 128 - WHITE_AB_TOL),
                            (255, 128 + WHITE_AB_TOL, 128 + WHITE_AB_TOL))

        best = None
        contours, _ = cv2.findContours(blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < MIN_AREA_PX or len(contour) < 5:
                continue
            (cx, cy), (a1, a2), _ = cv2.fitEllipse(contour)
            major, minor = max(a1, a2), min(a1, a2)
            if major <= 0 or minor / major < MIN_AXIS_RATIO:
                continue
            if area / (np.pi * major * minor / 4.0) < MIN_FILL_RATIO:
                continue
            inside = np.zeros(blue.shape, np.uint8)
            cv2.drawContours(inside, [contour], -1, 255, -1)
            digit_mask = cv2.bitwise_and(white, inside)
            if np.count_nonzero(digit_mask) / area < MIN_WHITE_RATIO:
                continue
            if best is None or area > best[0]:
                best = (area, contour, cx, digit_mask)

        result = None
        if best is not None:
            _, contour, cx, digit_mask = best
            normalized = normalize_digit(digit_mask)
            if capture:
                self.save_template(capture, normalized)
            digit, score = self.classify(normalized)
            result = (digit, score, contour)
            msg = StorageSign()
            msg.header = img.header
            msg.digit = digit
            msg.confidence = float(max(0.0, score))
            msg.image_x = float(cx / image.shape[1] * 2.0 - 1.0)
            msg.source = 'vorlage'
            self.pub_sign.publish(msg)
        elif capture:
            self.get_logger().warn('Vorlage nicht gespeichert: kein Schild im Bild.')
            self.set_parameters([Parameter('capture_digit', Parameter.Type.INTEGER, 0)])
        self.publish_debug(image, blue, result)

    def classify(self, normalized):
        """(Ziffer, Korrelation) der besten Vorlage; Ziffer 0, wenn nicht eindeutig."""
        if normalized is None:
            return 0, -1.0
        # Einfarbige Bilder ergeben NaN; als schlechtester Wert zaehlen.
        scores = sorted(
            ((float(np.nan_to_num(cv2.matchTemplate(normalized, tmpl, cv2.TM_CCOEFF_NORMED)[0, 0],
                                  nan=-1.0)), d)
             for d, tmpl in self.templates.items()), reverse=True)
        score, digit = scores[0]
        margin = score - scores[1][0] if len(scores) > 1 else score
        if score < MIN_SCORE or margin < MIN_MARGIN:
            return 0, score
        return digit, score

    def lab_clahe(self, image):
        l_channel, a_channel, b_channel = cv2.split(cv2.cvtColor(image, cv2.COLOR_BGR2LAB))
        clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP_LIMIT,
                                tileGridSize=(CLAHE_TILE_GRID_SIZE, CLAHE_TILE_GRID_SIZE))
        return cv2.merge((clahe.apply(l_channel), a_channel, b_channel))

    def template_path(self, digit):
        return os.path.join(os.path.expanduser(TEMPLATE_DIR), f'digit_{digit}.png')

    def load_templates(self):
        templates = {}
        for digit in DIGITS:
            path = self.template_path(digit)
            tmpl = cv2.imread(path, cv2.IMREAD_GRAYSCALE) if os.path.isfile(path) else None
            if tmpl is not None and tmpl.shape[::-1] == TEMPLATE_SIZE:
                templates[digit] = tmpl
                self.get_logger().info(f'Vorlage {digit}: {path}')
            else:
                templates[digit] = synthetic_digit(digit)
                self.get_logger().info(f'Vorlage {digit}: gezeichnet (keine Datei {path})')
        return templates

    def save_template(self, digit, normalized):
        self.set_parameters([Parameter('capture_digit', Parameter.Type.INTEGER, 0)])
        if normalized is None:
            self.get_logger().warn('Vorlage nicht gespeichert: keine weisse Ziffer im Schild.')
            return
        path = self.template_path(digit)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        cv2.imwrite(path, normalized)
        self.templates[digit] = normalized
        self.get_logger().info(f'Vorlage {digit} gespeichert: {path}')

    def publish_debug(self, image, blue, result):
        if self.pub_debug.get_subscription_count() == 0:
            return
        view = image.copy()
        view[blue > 0] = (0.5 * view[blue > 0] + (127, 0, 0)).astype(np.uint8)
        if result is not None:
            digit, score, contour = result
            cv2.drawContours(view, [contour], -1, (0, 255, 0), 2)
            x, y, _, _ = cv2.boundingRect(contour)
            cv2.putText(view, f'{digit or "?"} ({score:.2f})', (x, max(20, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)
        ok, encoded = cv2.imencode('.jpg', view)
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = array.array('B', encoded.tobytes())
        self.pub_debug.publish(msg)

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down storage_sign_node.')


def main(args=None):
    run_node(storage_sign_node, args)


if __name__ == '__main__':
    main()
