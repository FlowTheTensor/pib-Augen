import depthai as dai
import cv2

pipeline = dai.Pipeline()
camera = pipeline.create(dai.node.ColorCamera)

camera.setPreviewSize(640, 480)
camera.setInterleaved(False)
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("preview")
camera.preview.link(xout.input)

with dai.Device(pipeline) as device:
    q = device.getOutputQueue(name="preview", maxSize=4, blocking=False)
    while True:
        in_frame = q.get()
        frame = in_frame.getCvFrame()
        cv2.imshow("Kamera", frame)
        if cv2.waitKey(1) == ord('q'):
            break
cv2.destroyAllWindows()