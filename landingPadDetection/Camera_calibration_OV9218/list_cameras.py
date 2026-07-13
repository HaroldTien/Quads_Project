"""Probe camera indices 0-3 and report which ones deliver frames.

Run this on macOS to find the index of your built-in camera, then set
CAMERA_INDEX in vision_detection_lowLight.py to the number that works.
"""
import cv2

for idx in range(4):
    cap = cv2.VideoCapture(idx)          # default backend (AVFoundation on macOS)
    if not cap.isOpened():
        print(f"index {idx}: could not open")
        continue
    ret, frame = cap.read()
    if ret and frame is not None:
        h, w = frame.shape[:2]
        print(f"index {idx}: OK  -> frame {w}x{h}")
    else:
        print(f"index {idx}: opened but no frame")
    cap.release()
