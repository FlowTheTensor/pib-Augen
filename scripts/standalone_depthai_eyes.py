#!/usr/bin/env python3
"""
DepthAI preview (OAK-D Lite) without ROS.
Shows the live camera stream in a window.
"""

import argparse

import cv2
import depthai as dai


def build_pipeline(width: int, height: int, fps: float) -> dai.Pipeline:
    pipeline = dai.Pipeline()
    cam = pipeline.create(dai.node.Camera)
    cam.setFps(float(fps))
    cam.setSize((int(width), int(height)))
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("preview")
    cam.video.link(xout.input)
    return pipeline


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="DepthAI camera preview")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=400)
    parser.add_argument("--fps", type=float, default=30.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    pipeline = build_pipeline(args.width, args.height, args.fps)

    with dai.Device(pipeline) as device:
        queue = device.getOutputQueue("preview", maxSize=1, blocking=False)

        window_name = "OAK-D Preview - Press 'q' to quit"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        while True:
            packet = queue.tryGet()
            if packet is None:
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            frame = packet.getCvFrame()
            cv2.imshow(window_name, frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()