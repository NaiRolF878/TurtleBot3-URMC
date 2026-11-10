"""Kontakt und Griff: Bildvergleich direkt ueber dem Magneten.

Ein:  /lokal/raw_image (entzerrt), /control/storage_state
Aus:  /detect/pallet_still (Float32, Korrelation mit dem Bild von vor STILL_LAG_S), nur beim
      ANDOCKEN: steht das Bild still, obwohl der Bot vorfaehrt, liegt der Magnet an und
      schiebt die Palette.
      /detect/pallet_grip (Float32, Korrelation mit der Vorlage vom Greifen), nur in ATTACH_STATES,
      /detect/pallet_debug/compressed (nur mit Zuschauer)
"""
import array
import time
from collections import deque

import cv2
import numpy as np

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, String

from Follow_Lane.src.common import StorageState, run_node, storage_state_from_msg

# Lage des Magneten im entzerrten Bild (Anteil von Breite bzw. Hoehe: Mitte, Oberkante des
# Halters). Bot-Mechanik, kein Aufbau vor Ort: gemessen 10.11.2026 im Kamerabild; nur neu, wenn
# Kamera oder Magnet umgebaut werden. Aus der Geometrie (Magnetmitte 7,5 cm hoch) laesst es sich
# nicht rechnen, die Lage vor der Achse ist nur aus der BEV bekannt und dort verzerrt.
MAGNET_X = 0.49
MAGNET_TOP = 0.72
# Bildausschnitt ueber dem Magneten (Anteil von Bildbreite bzw. -hoehe).
ROI_WIDTH = 0.3
ROI_HEIGHT = 0.25
# So weit (px) darf sich der Ausschnitt verschieben, ohne dass der Vergleich leidet.
SEARCH_PAD_PX = 12
BLUR_KERNEL = 5
# Magnet an: dieses Bild wird zur Vorlage. Danach wird verglichen.
REF_STATE = StorageState.GREIFEN
ATTACH_STATES = (StorageState.GRIFF_PRUEFEN, StorageState.HERAUSFAHREN, StorageState.TRANSPORT,
                 StorageState.ABLAGE_ANFAHREN)
# Andocken: mit dem Bild von vor so vielen Sekunden vergleichen (Andocken mit ~3 cm/s, also
# ~1 cm: kommt die Palette naeher, aendert sich der Ausschnitt deutlich; schiebt der Bot sie
# schon, nicht).
STILL_STATE = StorageState.ANDOCKEN
STILL_LAG_S = 0.3


class pallet_node(Node):
    """Die Kamera sieht den Magneten; seine Lage im Bild steht in MAGNET_X und MAGNET_TOP.

    Kontakt (ANDOCKEN): der Metallstreifen ist buendig mit der Stuetze; liegt der Magnet an,
    schiebt der Bot die Palette vor sich her, sie bewegt sich mit ihm und der Ausschnitt steht
    still (/detect/pallet_still nahe 1). Vorher kommt sie naeher und er aendert sich.

    Griff: beim Greifen wird der Ausschnitt zur Vorlage. Haengt die Palette am Magneten, bewegt sie sich mit dem Bot und der Ausschnitt
    bleibt gleich (Korrelation nahe 1). Bleibt sie liegen, wird sie beim Zuruecksetzen kleiner
    und die Korrelation faellt ab. Ob das reicht, entscheidet switch_control.
    """

    def __init__(self):
        super().__init__('pallet_node')

        # --- Subscriber ---
        self.sub_state = self.create_subscription(
            String, '/control/storage_state', self.cb_state, 1)
        self.sub_image = self.create_subscription(Image, '/lokal/raw_image', self.cb_image, 1)

        # --- Publisher ---
        self.pub_grip = self.create_publisher(Float32, '/detect/pallet_grip', 1)
        self.pub_still = self.create_publisher(Float32, '/detect/pallet_still', 1)
        self.pub_debug = self.create_publisher(CompressedImage, '/detect/pallet_debug/compressed', 1)

        # --- Zustand ---
        self.bridge = CvBridge()
        self.state = StorageState.IDLE
        self.reference = None
        self._recent = deque()      # (Zeit, Ausschnitt) beim Andocken

        self.get_logger().info('Started pallet_node.')

    # --- Callbacks ---

    def cb_state(self, msg):
        self.state = storage_state_from_msg(msg)
        if self.state not in ATTACH_STATES and self.state != REF_STATE:
            self.reference = None
        if self.state != STILL_STATE:
            self._recent.clear()

    def cb_image(self, img):
        active = self.state in (REF_STATE, STILL_STATE) or self.state in ATTACH_STATES
        if not active and self.pub_debug.get_subscription_count() == 0:
            return
        try:
            self.check(img)
        except Exception as e:  # noqa: BLE001 -- ein kaputtes Bild darf den Node nicht stoppen
            self.get_logger().error(f'Griffpruefung fehlgeschlagen: {e}')

    # --- Hilfsmethoden ---

    def check(self, img):
        image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        gray = cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),
                                (BLUR_KERNEL, BLUR_KERNEL), 0)
        x0, y0, x1, y1 = self.roi(gray.shape)
        score = None
        if self.state == STILL_STATE:
            score = self.stillness(gray[y0:y1, x0:x1])
            if score is not None:
                self.pub_still.publish(Float32(data=score))
        elif self.state == REF_STATE:
            self.reference = gray[y0:y1, x0:x1].copy()
        elif self.reference is not None:
            h, w = gray.shape
            search = gray[max(0, y0 - SEARCH_PAD_PX):min(h, y1 + SEARCH_PAD_PX),
                          max(0, x0 - SEARCH_PAD_PX):min(w, x1 + SEARCH_PAD_PX)]
            result = cv2.matchTemplate(search, self.reference, cv2.TM_CCOEFF_NORMED)
            score = float(np.nan_to_num(result.max(), nan=-1.0))
            if self.state in ATTACH_STATES:
                self.pub_grip.publish(Float32(data=score))
        self.publish_debug(image, (x0, y0, x1, y1), score)

    def stillness(self, roi):
        """Korrelation des Ausschnitts mit dem von vor STILL_LAG_S; None, solange keiner so alt ist."""
        now = time.monotonic()
        self._recent.append((now, roi.copy()))
        old = None
        while self._recent and now - self._recent[0][0] >= STILL_LAG_S:
            old = self._recent.popleft()
        if old is None:
            return None
        self._recent.appendleft(old)    # juengstes ausreichend altes Bild bleibt Bezug
        result = cv2.matchTemplate(roi, old[1], cv2.TM_CCOEFF_NORMED)
        return float(np.nan_to_num(result.max(), nan=-1.0))

    def roi(self, shape):
        h, w = shape[:2]
        cx, bottom = MAGNET_X * w, MAGNET_TOP * h
        x0 = int(max(0, cx - ROI_WIDTH * w / 2.0))
        x1 = int(min(w, cx + ROI_WIDTH * w / 2.0))
        y0 = int(max(0, bottom - ROI_HEIGHT * h))
        return x0, y0, x1, int(bottom)

    def publish_debug(self, image, box, score):
        if self.pub_debug.get_subscription_count() == 0:
            return
        view = image.copy()
        x0, y0, x1, y1 = box
        cv2.rectangle(view, (x0, y0), (x1, y1), (0, 255, 255), 2)
        w = view.shape[1]
        cv2.line(view, (0, y1), (w - 1, y1), (0, 255, 255), 1)
        if score is None:
            text = 'Vorlage' if self.state == REF_STATE else 'keine Vorlage'
        elif self.state == STILL_STATE:
            text = f'Stillstand {score:.2f}'
        else:
            text = f'Korrelation {score:.2f}'
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
