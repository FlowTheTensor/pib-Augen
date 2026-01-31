from setuptools import setup

package_name = "eyes_display"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/eyes_display"]),
        ("share/eyes_display", ["package.xml"]),
        ("share/eyes_display/launch", ["launch/eyes_display.launch.py"]),
        ("share/eyes_display/config", ["config/eyes_display.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Full-screen 3D eyes display for a humanoid robot with ROS2 tracking input.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "eyes_node = eyes_display.eyes_node:main",
            "mock_publisher = eyes_display.mock_publisher:main",
            "face_tracker = eyes_display.face_tracker_node:main",
            "camera_bridge = eyes_display.camera_bridge_node:main",
        ],
    },
)
