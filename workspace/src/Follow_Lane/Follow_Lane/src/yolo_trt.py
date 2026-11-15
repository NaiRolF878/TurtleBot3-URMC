"""YOLO-Inferenz (Ultralytics-Export, ohne NMS im Modell) mit TensorRT auf der GPU.

GPU-Speicher und Kopien ueber cuda-python (cuda.bindings.runtime), ohne torch.

Die .engine muss mit derselben TensorRT-Version gebaut sein (Container: 10.15), sonst
laedt sie nicht. Neu bauen aus der ONNX: siehe docs/Befehle.md "YOLO-Engine bauen".
Ultralytics-.engine-Dateien (JSON-Metadaten vor der Engine) werden auch gelesen.
"""
import json

import cv2
import numpy as np
import tensorrt as trt
from cuda.bindings import runtime as cudart

# Klassen der Modelle Yolo/yolo26s_800 und yolo26s_640_data1311 (gleich, aus den Metadaten von best.engine). Falls die
# Engine eigene Metadaten hat, gelten die.
CLASS_NAMES = ['tb', 'park', 'tunnel', 'stop', 'left', 'right', 'work', 'start_line',
               'no_entry']
# Fuellfarbe der Raender beim Letterbox (wie Ultralytics).
LETTERBOX_FILL = 114


class _Cuda:
    """Das Noetigste aus der CUDA-Runtime-API (cuda-python): Stream, Speicher, Kopien."""

    def __init__(self):
        self._check(cudart.cudaSetDevice(0), 'cudaSetDevice')
        self.stream = self._check(cudart.cudaStreamCreate(), 'cudaStreamCreate')

    @staticmethod
    def _check(res, what):
        err, *rest = res if isinstance(res, tuple) else (res,)
        if err != cudart.cudaError_t.cudaSuccess:
            raise RuntimeError(f'{what} fehlgeschlagen ({err})')
        return rest[0] if len(rest) == 1 else (rest or None)

    def make_current(self):
        self._check(cudart.cudaSetDevice(0), 'cudaSetDevice')

    def alloc(self, nbytes):
        return int(self._check(cudart.cudaMalloc(nbytes), 'cudaMalloc'))

    def free(self, ptr):
        cudart.cudaFree(ptr)

    def to_device(self, dptr, host):
        self._check(cudart.cudaMemcpyAsync(
            dptr, host.ctypes.data, host.nbytes,
            cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, self.stream), 'cudaMemcpy HtoD')

    def to_host(self, host, dptr):
        self._check(cudart.cudaMemcpyAsync(
            host.ctypes.data, dptr, host.nbytes,
            cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost, self.stream), 'cudaMemcpy DtoH')

    def sync(self):
        self._check(cudart.cudaStreamSynchronize(self.stream), 'cudaStreamSynchronize')


def read_engine_file(engine_path):
    """Engine-Bytes und Ultralytics-Metadaten (dict, leer ohne). Ultralytics schreibt
    4 Byte Laenge + JSON-Metadaten vor die Engine."""
    with open(engine_path, 'rb') as f:
        data = f.read()
    n = int.from_bytes(data[:4], 'little')
    if 0 < n < 100000 and data[4:5] == b'{':
        try:
            return data[4 + n:], json.loads(data[4:4 + n])
        except ValueError:
            pass
    return data, {}


class TrtModel:
    """TensorRT-Engine mit einem Eingang (1, 3, H, W) und festen Ein-/Ausgabepuffern.
    Nicht threadsicher: immer aus demselben Thread aufrufen."""

    def __init__(self, engine_path):
        self.cuda = _Cuda()
        data, self.meta = read_engine_file(engine_path)
        self._logger = trt.Logger(trt.Logger.WARNING)
        self._runtime = trt.Runtime(self._logger)
        self.engine = self._runtime.deserialize_cuda_engine(data)
        if self.engine is None:
            raise RuntimeError(f'{engine_path} laedt nicht (andere TensorRT-Version? '
                               f'Container hat {trt.__version__})')
        self.context = self.engine.create_execution_context()

        self._buffers = {}
        self.out_names = []
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(name))
            host = np.empty(shape, trt.nptype(self.engine.get_tensor_dtype(name)))
            dptr = self.cuda.alloc(host.nbytes)
            self.context.set_tensor_address(name, dptr)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self._in_name = name
            else:
                self.out_names.append(name)
            self._buffers[name] = (host, dptr)
        _, _, self.in_h, self.in_w = self._buffers[self._in_name][0].shape

    def output_shape(self, name):
        return self._buffers[name][0].shape

    def infer(self, chw):
        """chw: (3, in_h, in_w) float. Gibt {Ausgang: Array ohne Batch-Achse} zurueck
        (Kopien, der naechste Aufruf ueberschreibt sie nicht)."""
        host_in, d_in = self._buffers[self._in_name]
        host_in[0] = chw.astype(host_in.dtype, copy=False)
        self.cuda.make_current()
        self.cuda.to_device(d_in, host_in)
        if not self.context.execute_async_v3(int(self.cuda.stream)):
            raise RuntimeError('TensorRT execute_async_v3 fehlgeschlagen')
        for name in self.out_names:
            host_out, d_out = self._buffers[name]
            self.cuda.to_host(host_out, d_out)
        self.cuda.sync()
        return {name: self._buffers[name][0][0].astype(np.float32) for name in self.out_names}

    def close(self):
        for _, dptr in self._buffers.values():
            self.cuda.free(dptr)
        self._buffers = {}


class YoloTRT(TrtModel):
    """detect(bgr) -> Liste (x0, y0, x1, y1, score, class_id, name) in Pixeln des
    uebergebenen Bilds. Nicht threadsicher: immer aus demselben Thread aufrufen."""

    kind = 'YOLO'

    def __init__(self, engine_path, conf=0.25, iou=0.7):
        super().__init__(engine_path)
        self.conf = conf
        self.iou = iou
        # Ein Ein- und ein Ausgang: images (1, 3, H, W), output0 (1, 4 + Klassen, Anker).
        self._out_name = self.out_names[0]
        names = self.meta.get('names')
        if isinstance(names, dict):
            self.names = [names[k] for k in sorted(names, key=int)]
        else:
            self.names = list(CLASS_NAMES)
        n_cls = self.output_shape(self._out_name)[1] - 4
        if n_cls != len(self.names):
            self.names = [str(i) for i in range(n_cls)]

    def letterbox(self, img):
        """Seitenverhaeltnis halten, mittig auf (in_h, in_w) auffuellen. Gibt Bild,
        Massstab und Rand (links, oben) zurueck."""
        h, w = img.shape[:2]
        r = min(self.in_h / h, self.in_w / w)
        nw, nh = int(round(w * r)), int(round(h * r))
        if (nw, nh) != (w, h):
            img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        dw, dh = (self.in_w - nw) / 2, (self.in_h - nh) / 2
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT,
                                 value=(LETTERBOX_FILL,) * 3)
        return img, r, left, top

    def detect(self, bgr):
        img, r, pad_x, pad_y = self.letterbox(bgr)
        # BGR -> RGB, HWC -> CHW, 0..1
        out = self.infer(img[:, :, ::-1].transpose(2, 0, 1) / 255.0)
        return self._postprocess(out[self._out_name], r, pad_x, pad_y, bgr.shape)

    def _postprocess(self, out, r, pad_x, pad_y, shape):
        # out: (4 + Klassen, Anker); Box = Mitte x/y, Breite, Hoehe im Letterbox-Bild,
        # Klassenwerte schon mit Sigmoid.
        scores_all = out[4:]
        cls = scores_all.argmax(axis=0)
        scores = scores_all[cls, np.arange(out.shape[1])]
        keep = scores >= self.conf
        if not keep.any():
            return []
        cx, cy, bw, bh = out[:4, keep]
        scores, cls = scores[keep], cls[keep]
        x0 = (cx - bw / 2 - pad_x) / r
        y0 = (cy - bh / 2 - pad_y) / r
        w, h = bw / r, bh / r

        # NMS je Klasse in einem Aufruf: Boxen jeder Klasse weit auseinanderschieben.
        offset = cls * 10000.0
        boxes = np.stack([x0 + offset, y0, w, h], axis=1).tolist()
        idx = cv2.dnn.NMSBoxes(boxes, scores.tolist(), self.conf, self.iou)
        idx = np.array(idx).reshape(-1)

        img_h, img_w = shape[:2]
        result = []
        for i in idx:
            bx0 = float(np.clip(x0[i], 0, img_w - 1))
            by0 = float(np.clip(y0[i], 0, img_h - 1))
            bx1 = float(np.clip(x0[i] + w[i], 0, img_w - 1))
            by1 = float(np.clip(y0[i] + h[i], 0, img_h - 1))
            c = int(cls[i])
            result.append((bx0, by0, bx1, by1, float(scores[i]), c, self.names[c]))
        return result
