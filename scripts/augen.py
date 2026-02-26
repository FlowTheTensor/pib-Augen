
import math
import time
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *


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

        pygame.init()
        display_flags = DOUBLEBUF | OPENGL
        if self.fullscreen:
            display_info = pygame.display.Info()
            self.width, self.height = display_info.current_w, display_info.current_h
            display_flags |= pygame.FULLSCREEN
        pygame.display.set_mode((self.width, self.height), display_flags)
        gluPerspective(45, self.width / self.height, 0.1, 100.0)
        glTranslatef(0.0, 0.0, -6.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        light_pos = (GLfloat * 4)(0.0, 2.0, 5.0, 1.0)
        glLightfv(GL_LIGHT0, GL_POSITION, light_pos)
        glClearColor(1.0, 1.0, 1.0, 1.0)
        self.quadric = gluNewQuadric()

    def on_draw(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, self.width / self.height, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -6.0)
        self.draw_eye(-1.7, 0.7, self.gx, self.gy)
        self.draw_eye(1.7, 0.7, self.gx, self.gy)
        pygame.display.flip()

    def draw_eye(self, cx, cy, gx, gy):
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
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, (GLfloat * 4)(0.9, 0.9, 0.9, 1.0))
        gluSphere(self.quadric, eyeball_radius, 40, 40)

        glPushMatrix()
        glTranslatef(iris_x, iris_y, iris_z)
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, (GLfloat * 4)(0.35, 0.05, 0.05, 1.0))
        gluSphere(self.quadric, iris_radius, 30, 30)
        glTranslatef(0.0, 0.0, 0.1)
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, (GLfloat * 4)(0.0, 0.0, 0.0, 1.0))
        gluSphere(self.quadric, pupil_radius, 20, 20)
        glPopMatrix()

        if self.blink > 0.0:
            lid_color = (GLfloat * 4)(0.8, 0.8, 0.8, 1.0)
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, lid_color)
            lid_radius = eyeball_radius * 1.02
            theta = self.blink * (math.pi / 2.0)
            glPushMatrix()
            glTranslatef(0.0, 0.0, eyeball_radius * 0.25)
            glDisable(GL_DEPTH_TEST)
            self.draw_sphere_segment(lid_radius, 0.0, theta)
            self.draw_sphere_segment(lid_radius, math.pi - theta, math.pi)
            glEnable(GL_DEPTH_TEST)
            glPopMatrix()
        glPopMatrix()

    def draw_sphere_segment(self, radius, theta_start, theta_end, slices=36, stacks=12):
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

    def update(self, _dt=0):
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
        gluDeleteQuadric(self.quadric)


def main():
    app = EyesDemo()
    clock = pygame.time.Clock()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        app.update()
        app.on_draw()
        clock.tick(app.fps)
    pygame.quit()

if __name__ == "__main__":
    main()
