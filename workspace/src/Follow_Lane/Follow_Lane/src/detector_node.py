"""Objekterkennung (RF-DETR/YOLO) in einem eigenen Prozess -- fuer das Rennen.

Ein:  /lokal/raw_image
Aus:  /detect/objects (Detection2DArray), /detect/start_line (Float32MultiArray),
      /detect/sign/<name> (Bool) -- dieselben Topics und Inhalte wie detect_lane.

Laeuft die Erkennung in detect_lane, blockiert sie dessen einzigen Thread: die Spurerkennung
fiel von 30 auf 18-20 Hz (17.11.). Hier laeuft sie in einem eigenen Prozess mit voller
Frequenz (jedes Rohbild; kommt waehrenddessen ein neues, gilt nur das neueste). detect_lane
laeuft dann mit detector: aus (Rennen.yaml). Gestartet von follow_lane.launch.py mit
detector_node:=true (rennen.sh).

Laden, Senden und Schilder wie in detect_lane (dieselben Methoden), damit beides gleich bleibt.
"""
import time

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray
from vision_msgs.msg import Detection2DArray

from Follow_Lane.src.common import frange, irange, read_params, require_params, run_node
from Follow_Lane.src.detect_lane_nav2 import SIGN_CLASSES, YOLO_MAX_AGE_S, detect_lane

# So oft (Bilder) Rechenzeit und Rate ins Log.
LOG_EVERY = 100


class detector_node(Node):
    def __init__(self):
        super().__init__('detector_node')
        self.declare_parameter('detector', Parameter.Type.STRING)
        self.declare_parameter('sign_min_conf', Parameter.Type.DOUBLE, frange(0.0, 1.0))
        self.declare_parameter('sign_min_height_px', Parameter.Type.INTEGER, irange(0, 600))
        self.declare_parameter('tunnel_min_height_px', Parameter.Type.INTEGER, irange(0, 600))
        self._param_names = ['detector', 'sign_min_conf', 'sign_min_height_px',
                             'tunnel_min_height_px']
        require_params(self, self._param_names)
        read_params(self, self._param_names)

        self.bridge = CvBridge()
        self.raw_image = None
        self.pub_objects = self.create_publisher(Detection2DArray, '/detect/objects', 1)
        self.pub_start_line = self.create_publisher(Float32MultiArray, '/detect/start_line', 1)
        self.pub_signs = {name: self.create_publisher(Bool, f'/detect/sign/{name}', 1)
                          for name in SIGN_CLASSES}
        self.model = detect_lane.load_detector(self, self.detector)
        self._times, self._t0 = [], time.monotonic()
        self.create_subscription(Image, '/lokal/raw_image', self.cb_image, 1)
        self.get_logger().info('Started detector_node.')

    def cb_image(self, img):
        if self.model is None:
            return
        read_params(self, ['sign_min_conf', 'sign_min_height_px', 'tunnel_min_height_px'])
        age = (self.get_clock().now() - Time.from_msg(img.header.stamp)).nanoseconds / 1e9
        if age > YOLO_MAX_AGE_S:
            return
        self.raw_image = img
        try:
            frame = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
            t = time.monotonic()
            detections = self.model.detect(frame)
            self._times.append((time.monotonic() - t) * 1000.0)
        except Exception as e:  # noqa: BLE001 -- ein kaputtes Bild darf den Node nicht stoppen
            self.get_logger().error(f'Erkennung fehlgeschlagen: {e}', throttle_duration_sec=5.0)
            return
        detect_lane.publish_detections(self, detections)
        detect_lane.publish_signs(self, detections)
        if len(self._times) >= LOG_EVERY:
            now = time.monotonic()
            self.get_logger().info(
                f'{self.model.kind}: {len(self._times) / (now - self._t0):.1f} Bilder/s, '
                f'{sum(self._times) / len(self._times):.1f} ms/Bild (max {max(self._times):.1f})')
            self._times, self._t0 = [], now

    def shut_down(self):
        self.get_logger().info('Shutting down detector_node.')
        if self.model is not None:
            self.model.close()


def main(args=None):
    run_node(detector_node, args)


if __name__ == '__main__':
    main()
