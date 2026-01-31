from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("eyes_display"), "config", "eyes_display.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="eyes_display",
                executable="eyes_node",
                name="eyes_display",
                output="screen",
                parameters=[config_path],
            ),
            Node(
                package="eyes_display",
                executable="mock_publisher",
                name="eyes_mock_publisher",
                output="screen",
                parameters=[{"tracking_topic": "/person/target"}],
            ),
        ]
    )
