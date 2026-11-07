"""Zonenschilder der Storage-Aufgabe per trainiertem Modell (Geruest).

Ein:  /lokal/raw_image (entzerrt), /control/storage_state
Aus:  /detect/storage_sign (StorageSign, source "modell"), nur im Zustand SCHILD_LESEN

Noch ohne Gewichte: ist model_path leer oder fehlt ultralytics, meldet der Node das
einmal und sendet nichts; switch_control stimmt dann nur mit storage_sign_node ab.
Erwartet ein YOLO-Modell (ultralytics) mit den Klassennamen aus CLASS_DIGITS.
"""
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import String

from follow_lane_msg.msg import StorageSign
from Follow_Lane.src.common import (
    StorageState, read_params, require_params, run_node, storage_state_from_msg)

# Klassenname -> Ziffer
CLASS_DIGITS = {'1': 1, '2': 2, '3': 3}
CONF_THRESHOLD = 0.6
IMAGE_SIZE = 320


class storage_sign_model_node(Node):

    def __init__(self):
        super().__init__('storage_sign_model_node')

        # --- Parameter ---
        self.declare_parameter('enabled', Parameter.Type.BOOL)
        self.declare_parameter('model_path', Parameter.Type.STRING)
        self._param_names = ['enabled', 'model_path']
        require_params(self, self._param_names)
        read_params(self, self._param_names)

        # --- Subscriber ---
        self.sub_state = self.create_subscription(
            String, '/control/storage_state', self.cb_state, 1)
        self.sub_image = self.create_subscription(Image, '/lokal/raw_image', self.cb_image, 1)

        # --- Publisher ---
        self.pub_sign = self.create_publisher(StorageSign, '/detect/storage_sign', 10)

        # --- Zustand ---
        self.bridge = CvBridge()
        self.state = StorageState.IDLE
        self.model = self.load_model()

        self.get_logger().info('Started storage_sign_model_node'
                               + ('.' if self.model else ' (ohne Modell, sendet nichts).'))

    # --- Callbacks ---

    def cb_state(self, msg):
        self.state = storage_state_from_msg(msg)

    def cb_image(self, img):
        if self.model is None or self.state != StorageState.SCHILD_LESEN:
            return
        read_params(self, self._param_names)
        if not self.enabled:
            return
        try:
            self.find_sign(img)
        except Exception as e:  # noqa: BLE001 -- ein kaputtes Bild darf den Node nicht stoppen
            self.get_logger().error(f'Modell fehlgeschlagen: {e}')

    # --- Hilfsmethoden ---

    def load_model(self):
        path = self.model_path
        if not path:
            self.get_logger().warn('model_path leer -- Modell aus.')
            return None
        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().warn('ultralytics nicht installiert -- Modell aus.')
            return None
        try:
            return YOLO(path)
        except Exception as e:  # noqa: BLE001 -- ohne Modell laeuft nur die Vorlagen-Erkennung
            self.get_logger().error(f'Modell {path} nicht ladbar: {e}')
            return None

    def find_sign(self, img):
        image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        result = self.model.predict(source=image, imgsz=IMAGE_SIZE, verbose=False)[0]
        if result.boxes is None or len(result.boxes) == 0:
            return
        # Nur die sicherste Erkennung; je Bild eine Stimme wie bei storage_sign_node.
        best = max(result.boxes, key=lambda b: float(b.conf[0]))
        conf = float(best.conf[0])
        name = result.names[int(best.cls[0])]
        if conf < CONF_THRESHOLD or name not in CLASS_DIGITS:
            return
        x0, _, x1, _ = (float(v) for v in best.xyxy[0])
        msg = StorageSign()
        msg.header = img.header
        msg.digit = CLASS_DIGITS[name]
        msg.confidence = conf
        msg.image_x = float((x0 + x1) / image.shape[1] - 1.0)
        msg.source = 'modell'
        self.pub_sign.publish(msg)

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down storage_sign_model_node.')


def main(args=None):
    run_node(storage_sign_model_node, args)


if __name__ == '__main__':
    main()
