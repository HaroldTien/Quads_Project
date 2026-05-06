import cv2
import time

# Arducam OV9281 with Jetvariety driver uses V4L2, not nvarguscamerasrc.
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

if not cap.isOpened():
    print("\nERROR: Cannot open camera on /dev/video0")
    print("How to fix:")
    print("1) Check Jetvariety driver is loaded:  lsmod | grep arducam")
    print("2) Verify device exists:  ls /dev/video*")
    print("3) Close any other app using the camera.")
    print("4) Re-seat the CSI cable and reboot if needed.")
    raise SystemExit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
cap.set(cv2.CAP_PROP_FPS, 30)

# Warm-up: OV9281 may need a few frames before delivering valid data.
for _ in range(30):
    ret, _ = cap.read()
    if ret:
        break
    time.sleep(0.02)

print("Camera ready!")
print("Preview window opened.")
print("Press ENTER or 'c' to capture, 'q' to quit.")
print("Goal: capture ~20 images from different angles.\n")

img_count = 0
grab_fail_count = 0

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        grab_fail_count += 1
        if grab_fail_count % 10 == 0:
            print(f"Warning: frame grab failed {grab_fail_count} times...")
        if grab_fail_count >= 60:
            print("Error: camera stream became unstable. Exiting.")
            break
        time.sleep(0.02)
        continue

    grab_fail_count = 0

    display = frame.copy()
    cv2.putText(
        display,
        f"Captured: {img_count} (target: 20) | ENTER/c: capture | q: quit",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    cv2.imshow("OV9281 - Calibration Capture", display)
    key = cv2.waitKey(1) & 0xFF

    if key in (10, 13, ord("c")):
        filename = f"calib_{img_count:02d}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Saved {filename} ✓")
        img_count += 1
        if img_count >= 20:
            print("\nGot 20 images! Press q to finish or keep capturing.")
    elif key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
print(f"\nDone! {img_count} calibration images saved.")
print("Next step: run the calibration script!")
