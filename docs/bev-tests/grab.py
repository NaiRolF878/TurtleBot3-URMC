#!/usr/bin/env python3
"""Holt je ein Bild von Rohkamera, entzerrtem Bild und Extrinsik-Ansicht und legt sie
mit Praefix ab. Aufruf: python3 docs/bev-tests/grab.py <praefix>"""
import os, sys, time
import cv2, numpy as np, rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from rclpy.qos import qos_profile_sensor_data

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), sys.argv[1])
rclpy.init(); n = rclpy.create_node('bev_test_grab'); br = CvBridge(); got = {}
def keep(key, conv):
    return lambda m: got.setdefault(key, (conv(m), len(getattr(m, 'data', b''))))
n.create_subscription(CompressedImage, '/camera/image_raw/compressed',
                      keep('roh', lambda m: cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)),
                      qos_profile_sensor_data)
n.create_subscription(Image, '/lokal/raw_image', keep('entzerrt', lambda m: br.imgmsg_to_cv2(m, 'bgr8')), 1)
n.create_subscription(Image, '/lokal/extrinsics_view', keep('ansicht', lambda m: br.imgmsg_to_cv2(m, 'bgr8')), 1)
end = time.time() + 15
while len(got) < 3 and time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.2)
for k, (img, nbytes) in got.items():
    cv2.imwrite(f'{out}_{k}.png', img)
    print(f'{out}_{k}.png  {img.shape[1]}x{img.shape[0]}' + (f'  JPEG {nbytes/1024:.0f} KB' if k == 'roh' else ''))
missing = {'roh', 'entzerrt', 'ansicht'} - set(got)
if missing: print('FEHLT:', missing)
