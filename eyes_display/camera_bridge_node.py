import base64
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from datatypes.srv import GetCameraImage
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraServiceBridge(Node):
    def __init__(self) -> None:
        super().__init__("camera_service_bridge")

        self.declare_parameter("service_name", "/get_camera_image")
        self.declare_parameter("output_topic", "/camera/image_raw")
        self.declare_parameter("poll_rate", 10.0)

        service_name = self.get_parameter("service_name").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        poll_rate = self.get_parameter("poll_rate").get_parameter_value().double_value

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, output_topic, 10)
        self.client = self.create_client(GetCameraImage, service_name)

        self.get_logger().info(f"Camera bridge waiting for service: {service_name}")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service not available, waiting...")

        period = 1.0 / max(0.1, float(poll_rate))
        self.timer = self.create_timer(period, self.on_timer)
        self.in_flight: Optional[rclpy.task.Future] = None

        self.get_logger().info(f"Publishing Image on {output_topic} at {poll_rate:.2f} Hz")

    def on_timer(self) -> None:
        if self.in_flight is not None and not self.in_flight.done():
            return

        request = GetCameraImage.Request()
        self.in_flight = self.client.call_async(request)
        self.in_flight.add_done_callback(self.on_response)

    def on_response(self, future: rclpy.task.Future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Service call failed: {exc}")
            return

        if not response or not response.image_base64:
            return

        try:
            img_bytes = base64.b64decode(response.image_base64)
            img_array = np.frombuffer(img_bytes, dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Decode failed: {exc}")
            return

        if frame is None:
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CameraServiceBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
