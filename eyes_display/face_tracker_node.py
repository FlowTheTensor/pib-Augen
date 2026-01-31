import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image


class FaceTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("face_tracker")

        self.declare_parameter("image_topic", "/oakd/rgb/image")
        self.declare_parameter("tracking_topic", "/person/target")
        self.declare_parameter("debug_topic", "/face_tracker/debug_image")
        self.declare_parameter("debug_enabled", True)
        self.declare_parameter("scale_factor", 1.1)
        self.declare_parameter("min_neighbors", 5)
        self.declare_parameter("min_size", 60)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        tracking_topic = self.get_parameter("tracking_topic").get_parameter_value().string_value
        debug_topic = self.get_parameter("debug_topic").get_parameter_value().string_value
        self.debug_enabled = self.get_parameter("debug_enabled").get_parameter_value().bool_value
        self.scale_factor = self.get_parameter("scale_factor").get_parameter_value().double_value
        self.min_neighbors = self.get_parameter("min_neighbors").get_parameter_value().integer_value
        self.min_size = self.get_parameter("min_size").get_parameter_value().integer_value

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Point, tracking_topic, 10)
        self.debug_pub = self.create_publisher(Image, debug_topic, 10)
        self.sub = self.create_subscription(Image, image_topic, self.on_image, 10)

        cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        if self.face_cascade.empty():
            self.get_logger().error(f"Failed to load Haar cascade: {cascade_path}")
        else:
            self.get_logger().info(f"Face tracker listening on {image_topic} -> {tracking_topic}")

    def on_image(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
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

        # largest face
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


def main() -> None:
    rclpy.init()
    node = FaceTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
