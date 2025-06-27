import numpy as np

class TrajectoryGenerator:
    def __init__(self):
        self.t = 0.0
        self.dt = 0.4

    def update(self, dt):
        self.t += dt
        x = 1.0 * np.cos(self.t)
        y = 1.0 * np.sin(self.t)
        z = -5.0
        dx = -1.0 * np.sin(self.t)
        dy = 1.0 * np.cos(self.t)
        dz = 0.0
        ddx = -1.0 * np.cos(self.t)
        ddy = -1.0 * np.sin(self.t)
        ddz = 0.0
        return np.array([[x, y, z], [dx, dy, dz], [ddx, ddy, ddz]])