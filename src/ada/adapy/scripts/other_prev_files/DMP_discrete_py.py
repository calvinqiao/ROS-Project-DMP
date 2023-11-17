from DMP_py import DMP
import numpy as np
import matplotlib.pyplot as plt


class DMPDiscrete(DMP):
    """
    An implementation of discrete DMP
    """
    def __init__(self, **kwargs):

        # call super class constructor
        super(DMPDiscrete, self).__init__(pattern='discrete', **kwargs)
        self.centers = None
        self.gen_centers()

        # set variance of Gaussian basis functions
        # trial and error to find this spacing
        self.h = np.ones(self.n_bfs) * self.n_bfs**1.5 / self.centers / self.cs.alpha_x

        self.check_offset()

    def gen_centers(self):
        """
        Set centers of the Gaussian basis functions throughout run time
        """

        # desired activation centers at time t throughout run time
        des_centers = np.linspace(0, self.cs.run_time, self.n_bfs)

        # initialize an array to store centers
        self.centers = np.zeros(len(des_centers))
        for n in range(len(des_centers)):
            # compute x for desired times t
            self.centers[n] = np.exp(-self.cs.alpha_x * des_centers[n])

    def gen_front_term(self, x):
        """
        Generate diminishing front term on the forcing term
        x float: current value of canonical system
        """
        return x * (self.y_goal[0] - self.y_init[0])

    def gen_goal(self, y_des):
        """
        Generate goal for path imitation
        y_des np.array: the desired trajectory to follow
        """
        return np.copy(y_des[:, -1])

    def gen_psi(self, x):
        """
        Generate the activity of basis functions for a given canonical system roll_out
        x float, array: the canonical system state
        """
        # print(x)
        if isinstance(x, np.ndarray):
            x = x[:, None]
        # psi(x)_i = exp(-h * (x-center_i)**2)
        return np.exp(-self.h * (x - self.centers)**2)

    def gen_weights(self, f_target):
        """
        Generate a set of weights over the basis functions
        such that the target forcing term trajectory is matched
        f_target np.array: the desired forcing term trajectory
        """

        # calculate x and psi throughout run time
        x_track = self.cs.compute_x_track()
        # print("In gen_weights")
        psi_track = self.gen_psi(x_track)
        # print(psi_track)

        # calculate Basis Function weights using weighted linear regression
        self.weights = np.zeros((1, self.n_bfs))
        # spatial scaling term
        spacial_scale = (self.y_goal[0] - self.y_init[0])
        for b in range(self.n_bfs):
            numerator = np.sum((spacial_scale * x_track) * psi_track[:, b] * f_target[:, 0])
            denominator = np.sum((spacial_scale * x_track)**2 * psi_track[:, b])
            self.weights[0, b] = numerator / denominator
        self.weights = np.nan_to_num(self.weights)