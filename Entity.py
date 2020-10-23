from abc import ABC
import numpy as np

class Entity(ABC):
    def __init__(self, color, thickness):
        self.color = color
        self.thickness = thickness

    def draw(self, canvas, c_rotmat, c_width, c_height):
        pass

class Point(Entity):
    def __init__(self, p, color, diameter):
        super().__init__(color, diameter)
        self.p = p

    def draw(self, canvas, c_rotmat, c_width, c_height):
        p_c = np.matmul(c_rotmat, self.p)
        p_image = np.array([p_c[0][0] + c_width/2, p_c[1][0] + c_height/2]).reshape(2, 1)

        x0 = p_image[0][0] - self.thickness/2
        y0 = p_image[1][0] - self.thickness/2
        x1 = p_image[0][0] + self.thickness/2
        y1 = p_image[1][0] + self.thickness/2
        canvas.create_oval(x0, y0, x1, y1, fill=self.color)

class Line(Entity):
    def __init__(self, p1, p2, color, thickness):
        super().__init__(color, thickness)
        self.p1 = p1
        self.p2 = p2

    def draw(self, canvas, c_rotmat, c_width, c_height):
        p1_c = np.matmul(c_rotmat, self.p1)
        p1_image = np.array([p1_c[0][0] + c_width/2, p1_c[1][0] + c_height/2]).reshape(2, 1)

        p2_c = np.matmul(c_rotmat, self.p2)
        p2_image = np.array([p2_c[0][0] + c_width/2, p2_c[1][0] + c_height/2]).reshape(2, 1)

        canvas.create_line(p1_image[0][0], p1_image[1][0],
                     p2_image[0][0], p2_image[1][0],
                     fill=self.color,
                     width=self.thickness)

class DottedLine(Entity):
    def __init__(self, p1, p2, color, thickness, num_segs):
        super().__init__(color, thickness)
        self.p1 = p1
        self.p2 = p2
        self.num_segs = num_segs

    def draw(self, canvas, c_rotmat, c_width, c_height):
        p_dist = self.p2 - self.p1
        interval = p_dist/(2*self.num_segs)
        intermediates = []
        for i in range(2*self.num_segs):
            intermediate_c = np.matmul(c_rotmat, self.p1 + i * interval)
            intermediates.append(np.array([intermediate_c[0][0] + c_width/2,
                                           intermediate_c[1][0] + c_height/2]).reshape(2, 1))

            if i % 2 != 0:
                canvas.create_line(intermediates[i][0][0], intermediates[i][1][0],
                         intermediates[i - 1][0][0], intermediates[i - 1][1][0],
                         fill=self.color,
                         width=self.thickness)