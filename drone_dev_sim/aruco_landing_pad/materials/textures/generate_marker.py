#!/usr/bin/env python3
"""Generate the ArUco landing-pad texture.

Spec (development landing pad):
  - Dictionary: DICT_5X5_50  (5x5 grid, 50-marker set -> largest inter-marker
    Hamming distance -> fewest false positives; best for a single marker)
  - Marker ID:  0
  - Marker size: 200 mm (A4 printable)
  - Plate:       280 mm  (adds a 40 mm white quiet zone around the marker so
                          the detector has clean contrast on every side)

The pixel layout maps 1:1 onto the plate in model.sdf:
  marker  = 800 px  -> 200 mm
  border  = 160 px each side
  plate   = 800 + 2*160 = 1120 px -> 280 mm
so the 800/1120 fraction of the 0.28 m plate is exactly 0.20 m of marker.

Re-run after changing any constant:
  python3 generate_marker.py
"""
import cv2
import cv2.aruco as aruco

DICTIONARY = aruco.DICT_5X5_50
MARKER_ID = 0
MARKER_PX = 800          # 200 mm
BORDER_PX = 160          # 40 mm quiet zone per side -> 280 mm plate
OUTPUT = "aruco_5x5_50_id0.png"

aruco_dict = aruco.getPredefinedDictionary(DICTIONARY)
marker = aruco.generateImageMarker(aruco_dict, MARKER_ID, MARKER_PX)

padded = cv2.copyMakeBorder(
    marker, BORDER_PX, BORDER_PX, BORDER_PX, BORDER_PX,
    cv2.BORDER_CONSTANT, value=255,
)

# Gazebo's PBR albedo pipeline expects a 3-channel image, not grayscale.
padded_bgr = cv2.cvtColor(padded, cv2.COLOR_GRAY2BGR)
cv2.imwrite(OUTPUT, padded_bgr)
print(f"wrote {OUTPUT}: {padded_bgr.shape[1]}x{padded_bgr.shape[0]} px "
      f"({MARKER_PX}px marker + {BORDER_PX}px border)")
