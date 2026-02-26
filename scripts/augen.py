import math
import time
import pyglet
from pyglet import gl
from pyglet.gl import glu

class EyesDemo:
    def __init__(self, width=800, height=480, fullscreen=True, fps=60):
        self.width = width
        self.height = height
        self.fullscreen = fullscreen
        self.fps = fps
        self.start_time = time.time()
        self.blink = 0.0
        self.gx = 0.0
        self.gy = 0.0

        if fullscreen:
            screen = pyglet.canvas.get_display().get_default_screen()
            self.width = screen.width
            self.height = screen.height

        self.window = pyglet.window.Window(
            width=self.width,
            height=self.height,
            fullscreen=self.fullscreen,
            caption="Augen-Demo",
            vsync=True,
        )
        self.window.push_handlers(self)
        self.quadric = glu.gluNewQuadric()

        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_LIGHTING)
        gl.glEnable(gl.GL_LIGHT0)
        light_pos = (gl.GLfloat * 4)(0.0, 2.0, 5.0, 1.0)
        gl.glLightfv(gl.GL_LIGHT0, gl.GL_POSITION, light_pos)
        gl.glClearColor(1.0, 1.0, 1.0, 1.0)

        pyglet.clock.schedule_interval(self.update, 1.0 / self.fps)

    def on_draw(self):
        self.window.clear()
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glViewport(0, 0, self.window.width, self.window.height)

        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        aspect = self.window.width / max(1.0, self.window.height)
        glu.gluPerspective(45.0, aspect, 0.1, 100.0)

        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()
        gl.glTranslatef(0.0, 0.0, -6.0)

        self.draw_eye(-1.7, 0.7, self.gx, self.gy)
        self.draw_eye(1.7, 0.7, self.gx, self.gy)

    def draw_eye(self, cx, cy, gx, gy):
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
        gl.glMaterialfv(gl.GL_FRONT_AND_BACK, gl.GL_AMBIENT_AND_DIFFUSE, (gl.GLfloat * 4)(0.9, 0.9, 0.9, 1.0))
        glu.gluSphere(self.quadric, eyeball_radius, 40, 40)

        gl.glPushMatrix()
        gl.glTranslatef(iris_x, iris_y, iris_z)
        gl.glMaterialfv(gl.GL_FRONT_AND_BACK, gl.GL_AMBIENT_AND_DIFFUSE, (gl.GLfloat * 4)(0.35, 0.05, 0.05, 1.0))
        glu.gluSphere(self.quadric, iris_radius, 30, 30)
        gl.glTranslatef(0.0, 0.0, 0.1)
        gl.glMaterialfv(gl.GL_FRONT_AND_BACK, gl.GL_AMBIENT_AND_DIFFUSE, (gl.GLfloat * 4)(0.0, 0.0, 0.0, 1.0))
        glu.gluSphere(self.quadric, pupil_radius, 20, 20)
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

    def draw_sphere_segment(self, radius, theta_start, theta_end, slices=36, stacks=12):
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

    def update(self, _dt):
        t = time.time() - self.start_time
        period = 5.0
        duration = 0.25
        phase = t % period
        if phase < duration:
            half = duration / 2.0
            self.blink = max(0.0, 1.0 - abs(phase - half) / half)
        else:
            self.blink = 0.0
        # Demo: Augen bewegen sich langsam im Kreis
        self.gx = math.sin(t * 0.5) * 0.7
        self.gy = math.cos(t * 0.3) * 0.5

    def on_close(self):
        glu.gluDeleteQuadric(self.quadric)


def main():
    app = EyesDemo()
    try:
        pyglet.app.run()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
