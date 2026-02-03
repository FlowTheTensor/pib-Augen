import argparse
import math
import os
import time

import cv2
import depthai as dai
import numpy as np
import pyglet

pyglet.options["shadow_window"] = False
if "PYGLET_PLATFORM" in os.environ:
    pyglet.options["platform"] = os.environ["PYGLET_PLATFORM"]

from pyglet import gl
from pyglet.gl import gluDeleteQuadric, gluNewQuadric, gluPerspective, gluSphere


class DepthAICamera:
    def __init__(self, width: int, height: int, fps: float) -> None:
        pipeline = dai.Pipeline()
        cam = pipeline.createColorCamera()
        cam.setPreviewSize(int(width), int(height))
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(float(fps))

        xout = pipeline.createXLinkOut()
        xout.setStreamName("preview")
        cam.preview.link(xout.input)

        self.device = dai.Device(pipeline)
        self.queue = self.device.getOutputQueue("preview", maxSize=1, blocking=False)

    def try_get_frame(self) -> np.ndarray | None:
        packet = self.queue.tryGet()
        if packet is None:
            return None
        return packet.getCvFrame()

    def close(self) -> None:
        try:
            self.device.close()
        except Exception:
            pass


class EyesApp:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.camera = DepthAICamera(args.preview_width, args.preview_height, args.fps)
        self.face_cascade = self._load_cascade()
        self.last_frame = None
        self.background_image = None
        self.background_size = None

        self.target = (0.0, 0.0)
        self.current = (0.0, 0.0)
        self.start_time = time.time()
        self.blink = 0.0

        width = args.screen_width
        height = args.screen_height
        if args.fullscreen:
            screen = pyglet.canvas.get_display().get_default_screen()
            width = screen.width
            height = screen.height

        self.window = pyglet.window.Window(
            width=width,
            height=height,
            fullscreen=args.fullscreen,
            caption="depthai_eyes",
            vsync=True,
        )
        self.window.push_handlers(self)

        self.quadric = gluNewQuadric()
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_LIGHTING)
        gl.glEnable(gl.GL_LIGHT0)
        light_pos = (gl.GLfloat * 4)(0.0, 2.0, 5.0, 1.0)
        gl.glLightfv(gl.GL_LIGHT0, gl.GL_POSITION, light_pos)
        gl.glClearColor(1.0, 1.0, 1.0, 1.0)

        pyglet.clock.schedule_interval(self.update, 1.0 / args.render_fps)

    def _load_cascade(self) -> cv2.CascadeClassifier:
        if hasattr(cv2, "data") and hasattr(cv2.data, "haarcascades"):
            path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
            if path:
                cascade = cv2.CascadeClassifier(path)
                if not cascade.empty():
                    return cascade

        for candidate in (
            "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
            "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
        ):
            cascade = cv2.CascadeClassifier(candidate)
            if not cascade.empty():
                return cascade

        raise RuntimeError("Haar cascade not found")

    def on_draw(self) -> None:
        self.window.clear()
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glViewport(0, 0, self.window.width, self.window.height)

        if self.background_image is not None:
            gl.glDisable(gl.GL_DEPTH_TEST)
            gl.glDisable(gl.GL_LIGHTING)
            gl.glMatrixMode(gl.GL_PROJECTION)
            gl.glLoadIdentity()
            gl.glOrtho(0, self.window.width, 0, self.window.height, -1, 1)
            gl.glMatrixMode(gl.GL_MODELVIEW)
            gl.glLoadIdentity()
            self.background_image.blit(0, 0, width=self.window.width, height=self.window.height)
            gl.glEnable(gl.GL_LIGHTING)
            gl.glEnable(gl.GL_DEPTH_TEST)

        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        aspect = self.window.width / max(1.0, self.window.height)
        gluPerspective(45.0, aspect, 0.1, 100.0)

        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()
        gl.glTranslatef(0.0, 0.0, -6.0)

        gx, gy = self.current
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

        gl.glPushMatrix()
        gl.glTranslatef(cx, cy, 0.0)

        gl.glMaterialfv(
            gl.GL_FRONT_AND_BACK,
            gl.GL_AMBIENT_AND_DIFFUSE,
            (gl.GLfloat * 4)(0.9, 0.9, 0.9, 1.0),
        )
        gluSphere(self.quadric, eyeball_radius, 40, 40)

        gl.glPushMatrix()
        gl.glTranslatef(iris_x, iris_y, iris_z)
        gl.glMaterialfv(
            gl.GL_FRONT_AND_BACK,
            gl.GL_AMBIENT_AND_DIFFUSE,
            (gl.GLfloat * 4)(0.35, 0.05, 0.05, 1.0),
        )
        gluSphere(self.quadric, iris_radius, 30, 30)

        gl.glTranslatef(0.0, 0.0, 0.1)
        gl.glMaterialfv(
            gl.GL_FRONT_AND_BACK,
            gl.GL_AMBIENT_AND_DIFFUSE,
            (gl.GLfloat * 4)(0.0, 0.0, 0.0, 1.0),
        )
        gluSphere(self.quadric, pupil_radius, 20, 20)
        gl.glPopMatrix()

        if self.blink > 0.0:
            lid_color = (gl.GLfloat * 4)(0.8, 0.8, 0.8, 1.0)
            gl.glMaterialfv(gl.GL_FRONT_AND_BACK, gl.GL_AMBIENT_AND_DIFFUSE, lid_color)

            lid_radius = eyeball_radius * 1.02
            theta = self.blink * (math.pi / 2.0)

            gl.glPushMatrix()
            gl.glTranslatef(0.0, 0.0, eyeball_radius * 0.25)
            gl.glDisable(gl.GL_DEPTH_TEST)
            self.draw_sphere_segment(lid_radius, 0.0, theta)
            self.draw_sphere_segment(lid_radius, math.pi - theta, math.pi)
            gl.glEnable(gl.GL_DEPTH_TEST)
            gl.glPopMatrix()

        gl.glPopMatrix()

    def draw_sphere_segment(
        self, radius: float, theta_start: float, theta_end: float, slices: int = 36, stacks: int = 12
    ) -> None:
        if theta_end <= theta_start:
            return

        for i in range(stacks):
            t0 = theta_start + (theta_end - theta_start) * (i / stacks)
            t1 = theta_start + (theta_end - theta_start) * ((i + 1) / stacks)
            gl.glBegin(gl.GL_QUAD_STRIP)
            for j in range(slices + 1):
                phi = 2.0 * math.pi * (j / slices)

                x0 = math.sin(t0) * math.cos(phi)
                y0 = math.cos(t0)
                z0 = math.sin(t0) * math.sin(phi)

                x1 = math.sin(t1) * math.cos(phi)
                y1 = math.cos(t1)
                z1 = math.sin(t1) * math.sin(phi)

                gl.glNormal3f(x0, y0, z0)
                gl.glVertex3f(radius * x0, radius * y0, radius * z0)
                gl.glNormal3f(x1, y1, z1)
                gl.glVertex3f(radius * x1, radius * y1, radius * z1)
            gl.glEnd()

    def update(self, _dt: float) -> None:
        frame = self.camera.try_get_frame()
        if frame is not None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(
                gray,
                scaleFactor=self.args.scale_factor,
                minNeighbors=self.args.min_neighbors,
                minSize=(self.args.min_size, self.args.min_size),
            )

            if len(faces) > 0:
                x, y, w, h = max(faces, key=lambda f: f[2] * f[3])
                cx = x + w / 2.0
                cy = y + h / 2.0
                height, width = gray.shape[:2]
                gx = (cx / width) * 2.0 - 1.0
                gy = 1.0 - (cy / height) * 2.0
                self.target = (
                    float(max(-1.0, min(1.0, gx))),
                    float(max(-1.0, min(1.0, gy))),
                )
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            flipped = np.flipud(rgb)
            height, width = rgb.shape[:2]
            self.background_image = pyglet.image.ImageData(
                width, height, "RGB", flipped.tobytes(), pitch=width * 3
            )
            self.background_size = (height, width)

        alpha = self.args.smoothing_alpha
        cx, cy = self.current
        tx, ty = self.target
        self.current = (cx + alpha * (tx - cx), cy + alpha * (ty - cy))

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
        self.camera.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Standalone DepthAI eyes display")
    parser.add_argument("--fullscreen", action="store_true", default=True)
    parser.add_argument("--windowed", action="store_true")
    parser.add_argument("--screen-width", type=int, default=800)
    parser.add_argument("--screen-height", type=int, default=480)
    parser.add_argument("--render-fps", type=float, default=60.0)
    parser.add_argument("--smoothing-alpha", type=float, default=0.2)

    parser.add_argument("--preview-width", type=int, default=640)
    parser.add_argument("--preview-height", type=int, default=400)
    parser.add_argument("--fps", type=float, default=30.0)

    parser.add_argument("--scale-factor", type=float, default=1.1)
    parser.add_argument("--min-neighbors", type=int, default=5)
    parser.add_argument("--min-size", type=int, default=60)

    args = parser.parse_args()
    if args.windowed:
        args.fullscreen = False
    return args


def main() -> None:
    args = parse_args()
    app = EyesApp(args)
    try:
        pyglet.app.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
