import cv2
import os

def gstreamer_pipeline(width=1280, height=720, fps=30, flip=0):
    return (
        f"nvarguscamerasrc ! "
        f"video/x-raw(memory:NVMM), "
        f"width={width}, height={height}, "
        f"framerate={fps}/1 ! "
        f"nvvidconv flip-method={flip} ! "
        f"video/x-raw, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! "
        f"appsink drop=1"
    )

cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("ERROR: Cannot open CSI camera!")
    exit()

print("Camera ready!")
print("Press ENTER to capture, type q + ENTER to quit")
print("Goal: capture ~20 images from different angles\n")

img_count = 0

while True:
    ret, frame = cap.read()

    if not ret or frame is None:
        print("Failed to grab frame!")
        break

    key = input(f"[{img_count}/20 captured] ENTER=capture, q=quit: ")

    if key == '':
        filename = f'calib_{img_count:02d}.jpg'
        cv2.imwrite(filename, frame)
        print(f"Saved {filename} ✓")
        img_count += 1

        if img_count >= 20:
            print("\nGot 20 images! Type q to finish or keep going for better accuracy.")

    elif key == 'q':
        break

cap.release()
print(f"\nDone! {img_count} calibration images saved.")
print("Next step: run the calibration script!")