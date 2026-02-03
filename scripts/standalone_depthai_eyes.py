#!/usr/bin/env python3
"""
Demo 1: Live camera preview with OpenCV.

What you’ll learn:
- How to open a webcam stream with cv2.VideoCapture
- How to read frames in a loop and display them with cv2.imshow
- How to handle keyboard input and exit cleanly
"""

import cv2

def open_camera(index: int = 0):
    """Try to open a camera by index and return the capture object.
    On Windows, DirectShow can be more reliable: cv2.VideoCapture(index, cv2.CAP_DSHOW).
    """
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        # Fallback for Windows users (harmless on other OSes; ignore if it doesn't help)
        cap.release()
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    return cap

def main():
    # 0 is usually the default webcam; try 1 if you have multiple cameras
    cap = open_camera(0)

    if not cap.isOpened():
        print("[ERROR] Could not open webcam. Try a different index (e.g., 1) or CAP_DSHOW on Windows.")
        return

    # (Optional) request a smaller resolution for better performance on older machines
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    window_name = "Live Camera - Press 'q' to quit"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)  # WINDOW_NORMAL lets you resize the window

    while True:
        # 1) Grab a frame
        ret, frame = cap.read()

        # 2) Safety check: if grabbing failed, stop the demo
        if not ret:
            print("[WARN] Frame grab failed. Exiting.")
            break

        # 3) Show the BGR frame (OpenCV uses BGR by default)
        cv2.imshow(window_name, frame)

        # 4) Keyboard handling: wait 1 ms and check if 'q' was pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 5) Cleanup: release camera and close windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()