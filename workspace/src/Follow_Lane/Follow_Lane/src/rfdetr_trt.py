"""RF-DETR-Inferenz (Roboflow-ONNX-Export) mit TensorRT auf der GPU, gleiche Schnittstelle
wie YoloTRT: detect(bgr) -> Liste (x0, y0, x1, y1, score, class_id, name).

Modell: Eingang 'input' (1, 3, S, S), Ausgaenge 'dets' (1, 300, 4) = Box Mitte x/y, Breite,
Hoehe als Anteil 0..1 des Bilds, 'labels' (1, 300, Klassen) = Logits (ohne Sigmoid).
Vorverarbeitung wie rfdetr: auf S x S strecken (kein Letterbox), RGB, 0..1, ImageNet-Norm.
DETR liefert je Abfrage eine Box, NMS braucht es nicht.

Engine bauen aus der ONNX: docs/Befehle.md "YOLO-/RF-DETR-Engine bauen".
"""
import cv2
import numpy as np

from Follow_Lane.src import yolo_trt
from Follow_Lane.src.yolo_trt import TrtModel

# Klasse je Ausgangsspalte: dieselbe Reihenfolge wie YOLO, dazu eine zehnte Spalte (rfdetr
# legt eine mehr an, sie kam nie vor). Abgeglichen 15.11. mit YOLO (Boxen gleicher Lage) auf
# dataset/*.jpg und den YOLO-Trainingsmosaiken: alle 9 Klassen eindeutig.
CLASS_NAMES = list(yolo_trt.CLASS_NAMES) + ['extra']
MEAN = np.array([0.485, 0.456, 0.406], np.float32).reshape(3, 1, 1)
STD = np.array([0.229, 0.224, 0.225], np.float32).reshape(3, 1, 1)


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


class RfDetrTRT(TrtModel):
    kind = 'RF-DETR'

    def __init__(self, engine_path, conf=0.5):
        super().__init__(engine_path)
        self.conf = conf
        if 'dets' not in self.out_names or 'labels' not in self.out_names:
            raise RuntimeError(f'{engine_path}: keine RF-DETR-Ausgaenge dets/labels '
                               f'({self.out_names})')
        n_cls = self.output_shape('labels')[2]
        self.names = list(CLASS_NAMES) if n_cls == len(CLASS_NAMES) \
            else [str(i) for i in range(n_cls)]

    def detect(self, bgr):
        img_h, img_w = bgr.shape[:2]
        img = cv2.resize(bgr, (self.in_w, self.in_h), interpolation=cv2.INTER_LINEAR)
        chw = img[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0
        out = self.infer((chw - MEAN) / STD)
        return self._postprocess(out['dets'], out['labels'], img_w, img_h)

    def _postprocess(self, dets, logits, img_w, img_h):
        probs = _sigmoid(logits)
        cls = probs.argmax(axis=1)
        scores = probs[np.arange(len(cls)), cls]
        result = []
        for i in np.flatnonzero(scores >= self.conf):
            cx, cy, bw, bh = dets[i]
            x0 = float(np.clip((cx - bw / 2) * img_w, 0, img_w - 1))
            y0 = float(np.clip((cy - bh / 2) * img_h, 0, img_h - 1))
            x1 = float(np.clip((cx + bw / 2) * img_w, 0, img_w - 1))
            y1 = float(np.clip((cy + bh / 2) * img_h, 0, img_h - 1))
            c = int(cls[i])
            result.append((x0, y0, x1, y1, float(scores[i]), c, self.names[c]))
        result.sort(key=lambda d: -d[4])
        return result
