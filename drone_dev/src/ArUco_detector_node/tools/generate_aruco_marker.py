#!/usr/bin/env python3
"""Generate a DICT_5X5_250 ArUco marker image for testing."""

import argparse
from pathlib import Path

import cv2


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate an ArUco marker PNG.")
    parser.add_argument("--id", type=int, default=0, help="Marker ID (0-249 for DICT_5X5_250).")
    parser.add_argument("--size", type=int, default=600, help="Output image size in pixels.")
    parser.add_argument(
        "--out",
        type=str,
        default="aruco_5x5_250_id0.png",
        help="Output PNG file path.",
    )
    args = parser.parse_args()

    if not (0 <= args.id <= 249):
        raise ValueError("For DICT_5X5_250, marker id must be between 0 and 249.")

    if args.size < 100:
        raise ValueError("Use a size >= 100 pixels for clear printing.")

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
    marker = cv2.aruco.generateImageMarker(dictionary, args.id, args.size)

    out_path = Path(args.out).expanduser().resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    ok = cv2.imwrite(str(out_path), marker)
    if not ok:
        raise RuntimeError(f"Failed to write marker image to: {out_path}")

    print(f"Saved marker: {out_path}")
    print(f"Dictionary: DICT_5X5_250 | ID: {args.id} | Size: {args.size}px")


if __name__ == "__main__":
    main()
