import math
import threading
import time
from dataclasses import dataclass
from typing import Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

import pyglet
from pyglet.gl import (
    GL_AMBIENT_AND_DIFFUSE,
    GL_COLOR_BUFFER_BIT,
    GL_DEPTH_BUFFER_BIT,
    GL_DEPTH_TEST,
    GL_DIFFUSE,
    GL_FRONT_AND_BACK,
    GL_LIGHT0,
    GL_LIGHTING,
    GL_MODELVIEW,
    GL_QUAD_STRIP,
    GL_POSITION,
    GL_PROJECTION,
    GLfloat,
    glClear,
    glClearColor,
    glColor3f,
    glEnable,
    glLightfv,
    glLoadIdentity,
    glMaterialfv,
    glMatrixMode,
    glBegin,
    glEnd,
    glNormal3f,
    glPopMatrix,
    glPushMatrix,
    glTranslatef,
    glVertex3f,
    glViewport,
)
from pyglet.gl import gluNewQuadric, gluPerspective, gluSphere, gluDeleteQuadric


@dataclass
class RenderParams:
    fullscreen: bool
    screen_width: int
    screen_height: int
    render_fps: float
    smoothing_alpha: float
    input_normalized: bool
    frame_width: float
    frame_height: float


class GazeState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._target = (0.0, 0.0)
        self._current = (0.0, 0.0)

    def set_target(self, x: float, y: float) -> None:
        with self._lock:
            self._target = (x, y)

    def get_current(self, alpha: float) -> Tuple[float, float]:
        with self._lock:
            tx, ty = self._target
            cx, cy = self._current
            nx = cx + alpha * (tx - cx)
            ny = cy + alpha * (ty - cy)
            self._current = (nx, ny)
            return self._current


class EyeTrackingNode(Node):
    def __init__(self, state: GazeState) -> None:
        super().__init__("eyes_display")
        self.state = state

        self.declare_parameter("tracking_topic", "/person/target")
        self.declare_parameter("input_normalized", True)
        self.declare_parameter("frame_width", 640.0)
        self.declare_parameter("frame_height", 480.0)

        topic = self.get_parameter("tracking_topic").get_parameter_value().string_value
        self.input_normalized = (
            self.get_parameter("input_normalized").get_parameter_value().bool_value
        )
        self.frame_width = (
            self.get_parameter("frame_width").get_parameter_value().double_value
        )
        self.frame_height = (
            self.get_parameter("frame_height").get_parameter_value().double_value
        )

        self.sub = self.create_subscription(Point, topic, self.on_point, 10)
        self.get_logger().info(f"Listening for tracking on {topic}")

    def on_point(self, msg: Point) -> None:
        if self.input_normalized:
            gx = max(-1.0, min(1.0, float(msg.x)))
            gy = max(-1.0, min(1.0, float(msg.y)))
        else:
            if self.frame_width <= 0 or self.frame_height <= 0:
                return
            gx = (float(msg.x) / self.frame_width) * 2.0 - 1.0
            gy = 1.0 - (float(msg.y) / self.frame_height) * 2.0
            gx = max(-1.0, min(1.0, gx))
            gy = max(-1.0, min(1.0, gy))

        self.state.set_target(gx, gy)


class EyesRenderer:
    def __init__(self, state: GazeState, params: RenderParams) -> None:
        self.state = state
        self.params = params

        width = params.screen_width
        height = params.screen_height
        if params.fullscreen:
            screen = pyglet.canvas.get_display().get_default_screen()
            width = screen.width
            height = screen.height

        self.window = pyglet.window.Window(
            width=width,
            height=height,
            fullscreen=params.fullscreen,
            caption="eyes_display",
            vsync=True,
        )
        self.window.push_handlers(self)
        self.quadric = gluNewQuadric()

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)

        light_pos = (GLfloat * 4)(0.0, 2.0, 5.0, 1.0)
        glLightfv(GL_LIGHT0, GL_POSITION, light_pos)
        glClearColor(1.0, 1.0, 1.0, 1.0)

        self.start_time = time.time()
        self.blink = 0.0

        pyglet.clock.schedule_interval(self.update, 1.0 / params.render_fps)

    def on_draw(self) -> None:
        self.window.clear()
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glViewport(0, 0, self.window.width, self.window.height)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = self.window.width / max(1.0, self.window.height)
        gluPerspective(45.0, aspect, 0.1, 100.0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -6.0)

        gx, gy = self.state.get_current(self.params.smoothing_alpha)
        self.draw_eye(-1.7, 0.7, gx, gy)
        self.draw_eye(1.7, 0.7, gx, gy)

    def draw_eye(self, cx: float, cy: float, gx: float, gy: float) -> None:
        scale = 1.2
        eyeball_radius = 1.0 * scale
        iris_radius = 0.45 * scale
        pupil_radius = 0.2 * scale

        gaze_scale = 0.35
        iris_x = gx * gaze_scale
        iris_y = gy * gaze_scale
        iris_z = 0.85

        glPushMatrix()
        glTranslatef(cx, cy, 0.0)

        # Eyeball
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, (GLfloat * 4)(0.9, 0.9, 0.9, 1.0))
        gluSphere(self.quadric, eyeball_radius, 40, 40)

        # Iris
        glPushMatrix()
        glTranslatef(iris_x, iris_y, iris_z)
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, (GLfloat * 4)(0.35, 0.05, 0.05, 1.0))
        gluSphere(self.quadric, iris_radius, 30, 30)

        # Pupil
        glTranslatef(0.0, 0.0, 0.1)
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, (GLfloat * 4)(0.0, 0.0, 0.0, 1.0))
        gluSphere(self.quadric, pupil_radius, 20, 20)
        glPopMatrix()

        # Eyelids (gray) as spherical caps with growing angle
        if self.blink > 0.0:
            lid_color = (GLfloat * 4)(0.6, 0.6, 0.6, 1.0)
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, lid_color)

            lid_radius = eyeball_radius * 1.02
            theta = self.blink * (math.pi / 2.0)

            glPushMatrix()
            glTranslatef(0.0, 0.0, eyeball_radius * 0.08)
            # Upper lid: from north pole to theta
            self.draw_sphere_segment(lid_radius, 0.0, theta)
            # Lower lid: from (pi - theta) to south pole
            self.draw_sphere_segment(lid_radius, math.pi - theta, math.pi)
            glPopMatrix()

        glPopMatrix()

    def draw_sphere_segment(
        self, radius: float, theta_start: float, theta_end: float, slices: int = 36, stacks: int = 12
    ) -> None:
        if theta_end <= theta_start:
            return

        for i in range(stacks):
            t0 = theta_start + (theta_end - theta_start) * (i / stacks)
            t1 = theta_start + (theta_end - theta_start) * ((i + 1) / stacks)
            glBegin(GL_QUAD_STRIP)
            for j in range(slices + 1):
                phi = 2.0 * math.pi * (j / slices)

                x0 = math.sin(t0) * math.cos(phi)
                y0 = math.cos(t0)
                z0 = math.sin(t0) * math.sin(phi)

                x1 = math.sin(t1) * math.cos(phi)
                y1 = math.cos(t1)
                z1 = math.sin(t1) * math.sin(phi)

                glNormal3f(x0, y0, z0)
                glVertex3f(radius * x0, radius * y0, radius * z0)
                glNormal3f(x1, y1, z1)
                glVertex3f(radius * x1, radius * y1, radius * z1)
            glEnd()

    def update(self, _dt: float) -> None:
        t = time.time() - self.start_time
        period = 5.0
        duration = 0.25
        phase = t % period
        if phase < duration:
            half = duration / 2.0
            self.blink = max(0.0, 1.0 - abs(phase - half) / half)
        else:
            self.blink = 0.0

    def on_close(self) -> None:
        gluDeleteQuadric(self.quadric)


def build_params(node: Node) -> RenderParams:
    node.declare_parameter("fullscreen", True)
    node.declare_parameter("screen_width", 800)
    node.declare_parameter("screen_height", 480)
    node.declare_parameter("render_fps", 60.0)
    node.declare_parameter("smoothing_alpha", 0.2)

    return RenderParams(
        fullscreen=node.get_parameter("fullscreen").get_parameter_value().bool_value,
        screen_width=node.get_parameter("screen_width").get_parameter_value().integer_value,
        screen_height=node.get_parameter("screen_height").get_parameter_value().integer_value,
        render_fps=node.get_parameter("render_fps").get_parameter_value().double_value,
        smoothing_alpha=node.get_parameter("smoothing_alpha").get_parameter_value().double_value,
        input_normalized=node.get_parameter("input_normalized").get_parameter_value().bool_value,
        frame_width=node.get_parameter("frame_width").get_parameter_value().double_value,
        frame_height=node.get_parameter("frame_height").get_parameter_value().double_value,
    )


def main() -> None:
    rclpy.init()
    state = GazeState()
    node = EyeTrackingNode(state)

    params = build_params(node)

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    renderer = EyesRenderer(state, params)

    try:
        pyglet.app.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
