import cv2

# Initialize the V4L2 backend explicitly for better performance on Jetson
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

if not cap.isOpened():
    print("Failed to open camera.")
    exit()

# Force the resolution and high framerate for the OV9281
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
cap.set(cv2.CAP_PROP_FPS, 120)

print("Streaming... Press 'q' to exit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Dropped frame!")
        break

    # The frame is already grayscale, but OpenCV often loads it into a 3-channel matrix by default.
    # For algorithms like ArUco marker detection, working with a single-channel image is mathematically faster.
    if len(frame.shape) == 3:
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        gray_frame = frame

    cv2.imshow("OV9281 High-Speed Feed", gray_frame)

    # Exit condition
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()