import os
import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import depthai as dai
except ImportError:  # pragma: no cover - optional dependency
    dai = None


class DepthAIFaceTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("depthai_face_tracker")

        self.declare_parameter("tracking_topic", "/person/target")
        self.declare_parameter("debug_topic", "/face_tracker/debug_image")
        self.declare_parameter("debug_enabled", True)
        self.declare_parameter("scale_factor", 1.1)
        self.declare_parameter("min_neighbors", 5)
        self.declare_parameter("min_size", 60)
        self.declare_parameter("preview_width", 640)
        self.declare_parameter("preview_height", 400)
        self.declare_parameter("fps", 30.0)

        tracking_topic = self.get_parameter("tracking_topic").get_parameter_value().string_value
        debug_topic = self.get_parameter("debug_topic").get_parameter_value().string_value
        self.debug_enabled = self.get_parameter("debug_enabled").get_parameter_value().bool_value
        self.scale_factor = self.get_parameter("scale_factor").get_parameter_value().double_value
        self.min_neighbors = self.get_parameter("min_neighbors").get_parameter_value().integer_value
        self.min_size = self.get_parameter("min_size").get_parameter_value().integer_value
        self.preview_width = self.get_parameter("preview_width").get_parameter_value().integer_value
        self.preview_height = self.get_parameter("preview_height").get_parameter_value().integer_value
        self.fps = self.get_parameter("fps").get_parameter_value().double_value

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Point, tracking_topic, 10)
        self.debug_pub = self.create_publisher(Image, debug_topic, 10)

        self.device = None
        self.queue = None
        self.failed = False

        cascade_path = self._resolve_cascade()
        if cascade_path is None:
            self.get_logger().error("Failed to locate Haar cascade file.")
            self.face_cascade = cv2.CascadeClassifier()
            self.failed = True
            self.create_timer(0.1, self._shutdown_if_failed)
            return

        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        if self.face_cascade.empty():
            self.get_logger().error(f"Failed to load Haar cascade: {cascade_path}")
            self.failed = True
            self.create_timer(0.1, self._shutdown_if_failed)
            return

        if dai is None:
            self.get_logger().error("depthai is not installed. Install with 'python3 -m pip install depthai'.")
            self.failed = True
            self.create_timer(0.1, self._shutdown_if_failed)
            return

        try:
            pipeline = dai.Pipeline()
            cam = pipeline.createColorCamera()
            cam.setPreviewSize(int(self.preview_width), int(self.preview_height))
            cam.setInterleaved(False)
            cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
            cam.setFps(float(self.fps))

            xout = pipeline.createXLinkOut()
            xout.setStreamName("preview")
            cam.preview.link(xout.input)

            self.device = dai.Device(pipeline)
            self.queue = self.device.getOutputQueue("preview", maxSize=1, blocking=False)
        except Exception as exc:
            self.get_logger().error(f"Failed to start DepthAI pipeline: {exc}")
            self.failed = True
            self.create_timer(0.1, self._shutdown_if_failed)
            return

        period = 1.0 / max(1.0, float(self.fps))
        self.timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(f"DepthAI face tracker publishing {tracking_topic}")

    def _resolve_cascade(self) -> str | None:
        if hasattr(cv2, "data") and hasattr(cv2.data, "haarcascades"):
            candidate = os.path.join(cv2.data.haarcascades, "haarcascade_frontalface_default.xml")
            if os.path.exists(candidate):
                return candidate

        for candidate in (
            "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
            "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
        ):
            if os.path.exists(candidate):
                return candidate

        return None

    def _shutdown_if_failed(self) -> None:
        if self.failed:
            rclpy.shutdown()

    def _on_timer(self) -> None:
        if self.queue is None:
            return

        packet = self.queue.tryGet()
        if packet is None:
            return

        frame = packet.getCvFrame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=float(self.scale_factor),
            minNeighbors=int(self.min_neighbors),
            minSize=(int(self.min_size), int(self.min_size)),
        )

        if len(faces) == 0:
            if self.debug_enabled:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
            return

        x, y, w, h = max(faces, key=lambda f: f[2] * f[3])
        cx = x + w / 2.0
        cy = y + h / 2.0

        height, width = gray.shape[:2]
        if width <= 0 or height <= 0:
            return

        gx = (cx / width) * 2.0 - 1.0
        gy = 1.0 - (cy / height) * 2.0

        msg_out = Point()
        msg_out.x = float(max(-1.0, min(1.0, gx)))
        msg_out.y = float(max(-1.0, min(1.0, gy)))
        msg_out.z = 0.0
        self.pub.publish(msg_out)

        if self.debug_enabled:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

    def destroy_node(self) -> bool:
        if self.device is not None:
            try:
                self.device.close()
            except Exception:
                pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = DepthAIFaceTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
