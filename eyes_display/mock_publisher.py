import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node


class MockPublisher(Node):
    def __init__(self) -> None:
        super().__init__("eyes_mock_publisher")
        self.declare_parameter("tracking_topic", "/person/target")
        topic = self.get_parameter("tracking_topic").get_parameter_value().string_value
        self.pub = self.create_publisher(Point, topic, 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.on_timer)
        self.get_logger().info(f"Publishing mock gaze to {topic}")

    def on_timer(self) -> None:
        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        msg = Point()
        msg.x = math.sin(t) * 0.7
        msg.y = math.cos(t * 0.8) * 0.4
        msg.z = 0.0
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MockPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
