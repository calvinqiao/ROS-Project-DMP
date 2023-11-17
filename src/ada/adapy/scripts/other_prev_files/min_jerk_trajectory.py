import numpy as np


class MinJerkTrajectory(object):

    def __init__(self, dt):
        self.dt = dt
        self.run_time = 1.0
        self.time_steps = int(self.run_time / self.dt)
        self.goal = 1.0
        self.tau = self.run_time*0.7

    def step(self, x, xd, xdd, tau):
        if tau < self.dt:
            return x, xd, xdd
        dist = self.goal - x

        a1 = 0
        a0 = xdd * (tau ** 2)
        v1 = 0
        v0 = xd * tau

        t1 = self.dt
        t2 = self.dt ** 2
        t3 = self.dt ** 3
        t4 = self.dt ** 4
        t5 = self.dt ** 5

        c1 = (6.0 * dist + (a1 - a0) / 2.0 - 3.0 * (v0 + v1)) / tau ** 5
        c2 = (-15.0 * dist + (3.0 * a0 - 2.0 * a1) / 2.0 + 8.0 * v0 + 7.0 * v1) / tau ** 4
        c3 = (10.0 * dist + (a1 - 3.0 * a0) / 2.0 - 6. * v0 - 4.0 * v1) / tau ** 3
        c4 = xdd / 2.0
        c5 = xd
        c6 = x

        x = c1 * t5 + c2 * t4 + c3 * t3 + c4 * t2 + c5 * t1 + c6
        xd = 5.0 * c1 * t4 + 4 * c2 * t3 + 3 * c3 * t2 + 2 * c4 * t1 + c5
        xdd = 20.0 * c1 * t3 + 12.0 * c2 * t2 + 6.0 * c3 * t1 + 2.0 * c4

        return x, xd, xdd

    def gen_trajectory(self):

        y_track = np.zeros((self.time_steps, 3))
        des_y = 0
        des_dy = 0
        des_ddy = 0
        for i in range(self.time_steps):
            des_y, des_dy, des_ddy = self.step(des_y, des_dy, des_ddy,
                                               self.tau - i*self.dt)
            y_track[i] = [des_y, des_dy, des_ddy]

        return y_track[:, 0]